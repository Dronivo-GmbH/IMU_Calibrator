/*
 * BMI160 serial streamer for IMU calibration GUI
 *
 * Hardware: ESP32 / Arduino + BMI160 I2C breakout (6-axis; mag streamed as 0)
 *
 * Required library (Arduino Library Manager):
 *   - ArduinoJson (by Benoit Blanchon) v6.x
 *
 * Wiring (ESP32):
 *   SDA -> GPIO21
 *   SCL -> GPIO22
 *   3.3V -> VCC
 *   GND  -> GND
 *
 * Serial: 115200 baud
 *
 * Protocol:
 *   Stream:  ax,ay,az,gx,gy,gz,mx,my,mz
 *   Commands: PING, CAL_MODE_ON, CAL_MODE_OFF, GET_CAL, SET_CAL {...}, I2C_SCAN
 */

#include <Wire.h>
#include <ArduinoJson.h>

#if defined(ESP32)
#include <Preferences.h>
#else
#include <EEPROM.h>
#endif

// ---------- BMI160 registers ----------
static const uint8_t REG_CHIP_ID = 0x00;
static const uint8_t REG_GYR_DATA = 0x0C;
static const uint8_t REG_ACC_DATA = 0x12;
static const uint8_t REG_ACC_CONF = 0x40;
static const uint8_t REG_ACC_RANGE = 0x41;
static const uint8_t REG_GYR_CONF = 0x42;
static const uint8_t REG_GYR_RANGE = 0x43;
static const uint8_t REG_PWR_CONF = 0x7C;
static const uint8_t REG_PMU_STATUS = 0x03;
static const uint8_t REG_PMU_TRIGGER = 0x6C;

static const uint8_t BMI160_CHIP_ID = 0xD1;
static const uint8_t CMD_SOFT_RESET = 0xB6;
static const uint8_t CMD_ACCEL_NORMAL = 0x11;
static const uint8_t CMD_GYRO_NORMAL = 0x15;
static const uint8_t REG_CMD = 0x7E;

// ---------- Config ----------
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t SAMPLE_HZ = 100;
static const uint32_t SAMPLE_MS = 1000 / SAMPLE_HZ;
static const uint32_t I2C_CLOCK_HZ = 100000;

static const float ACCEL_RANGE_G = 8.0f;
static const float GYRO_RANGE_DPS = 2000.0f;
static const float GRAVITY = 9.80665f;

static const uint32_t CAL_MAGIC = 0xCA1B1601;

// ---------- State ----------
uint8_t bmi160Addr = 0;
bool calibrationMode = false;
uint32_t lastSampleMs = 0;
uint32_t readFailCount = 0;
uint32_t zeroStreak = 0;

struct CalProfile {
  uint32_t magic;
  float gyro_offset[3];
  float accel_offset[3];
  float accel_scale[3];
  float mag_offset[3];
  float mag_soft_iron[3][3];
};

CalProfile cal;

void loadCalibration();
void saveCalibration();
void resetCalibrationDefaults();
void emitCalibrationJson();
bool parseAndApplyCalibration(const char *json);
void handleSerialCommand(String &line);
void streamSample();
float rawAccelToMS2(int16_t raw);
float rawGyroToDPS(int16_t raw);
void initI2C();
void scanI2CBus();
uint8_t findBmi160Address();
bool readBmi160ChipId(uint8_t addr, uint8_t &chipId);
bool bmiWrite8(uint8_t reg, uint8_t val);
bool bmiReadBytes(uint8_t reg, uint8_t *buf, uint8_t len);
bool bmi160Init();
bool bmi160EnsureSensorsOn();
bool bmi160ReadRaw(int16_t &gx, int16_t &gy, int16_t &gz, int16_t &ax, int16_t &ay, int16_t &az);
bool accelHasGravity(int16_t ax, int16_t ay, int16_t az);

void resetCalibrationDefaults() {
  cal.magic = CAL_MAGIC;
  for (int i = 0; i < 3; i++) {
    cal.gyro_offset[i] = 0.0f;
    cal.accel_offset[i] = 0.0f;
    cal.accel_scale[i] = 1.0f;
    cal.mag_offset[i] = 0.0f;
    for (int j = 0; j < 3; j++) {
      cal.mag_soft_iron[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

void loadCalibration() {
  resetCalibrationDefaults();

#if defined(ESP32)
  Preferences prefs;
  if (!prefs.begin("imu_cal", true)) {
    return;
  }
  if (prefs.getUInt("magic", 0) != CAL_MAGIC) {
    prefs.end();
    return;
  }
  prefs.getBytes("profile", &cal, sizeof(CalProfile));
  prefs.end();
#else
  EEPROM.get(0, cal);
  if (cal.magic != CAL_MAGIC) {
    resetCalibrationDefaults();
  }
#endif
}

void saveCalibration() {
  cal.magic = CAL_MAGIC;

#if defined(ESP32)
  Preferences prefs;
  if (!prefs.begin("imu_cal", false)) {
    return;
  }
  prefs.putUInt("magic", CAL_MAGIC);
  prefs.putBytes("profile", &cal, sizeof(CalProfile));
  prefs.end();
#else
  EEPROM.put(0, cal);
#endif
}

float rawAccelToMS2(int16_t raw) {
  return (raw * ACCEL_RANGE_G / 32768.0f) * GRAVITY;
}

float rawGyroToDPS(int16_t raw) {
  return raw * GYRO_RANGE_DPS / 32768.0f;
}

void initI2C() {
#if defined(ESP32)
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  Wire.begin(21, 22);
#else
  Wire.begin();
#endif
  Wire.setClock(I2C_CLOCK_HZ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setTimeout(50);
#endif
  delay(50);
}

bool readBmi160ChipId(uint8_t addr, uint8_t &chipId) {
  Wire.beginTransmission(addr);
  Wire.write(REG_CHIP_ID);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) {
    return false;
  }
  chipId = Wire.read();
  return true;
}

uint8_t findBmi160Address() {
  const uint8_t candidates[] = {0x68, 0x69};
  for (uint8_t addr : candidates) {
    uint8_t chipId = 0;
    if (readBmi160ChipId(addr, chipId) && chipId == BMI160_CHIP_ID) {
      return addr;
    }
  }
  return 0;
}

void scanI2CBus() {
  Serial.println("# I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("#   0x");
      if (addr < 16) {
        Serial.print('0');
      }
      Serial.print(addr, HEX);
      uint8_t chipId = 0;
      if (readBmi160ChipId(addr, chipId)) {
        Serial.print("  chip_id=0x");
        Serial.print(chipId, HEX);
      }
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println("#   (no devices)");
  }
}

bool bmiWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bmi160Addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool bmiReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(bmi160Addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t received = Wire.requestFrom(bmi160Addr, len);
  if (received == 0) {
    return false;
  }
  uint8_t i = 0;
  uint32_t start = millis();
  while (i < len && millis() - start < 25) {
    if (Wire.available()) {
      buf[i++] = Wire.read();
    }
  }
  return i == len;
}

bool bmi160EnsureSensorsOn() {
  uint8_t pmu = 0;
  if (!bmiReadBytes(REG_PMU_STATUS, &pmu, 1)) {
    return false;
  }

  bool ok = true;
  if ((pmu & 0x30) != 0x10) {
    ok = bmiWrite8(REG_CMD, CMD_ACCEL_NORMAL) && ok;
    delay(5);
  }
  if ((pmu & 0x0C) != 0x04) {
    ok = bmiWrite8(REG_CMD, CMD_GYRO_NORMAL) && ok;
    delay(5);
  }
  return ok;
}

bool accelHasGravity(int16_t ax, int16_t ay, int16_t az) {
  // Stationary accel should never be exactly 0 on all axes (gravity).
  return (ax != 0 || ay != 0 || az != 0);
}

bool bmi160Init() {
  uint8_t dummy = 0;
  bmiReadBytes(0x7F, &dummy, 1);

  if (!bmiWrite8(REG_CMD, CMD_SOFT_RESET)) {
    return false;
  }
  delay(100);

  bmiWrite8(REG_PMU_TRIGGER, 0x00);
  if (!bmiWrite8(REG_PWR_CONF, 0x00)) {
    return false;
  }
  delay(20);

  // Configure ranges/ODR before enabling sensors (Bosch recommended order).
  if (!bmiWrite8(REG_ACC_CONF, 0x28)) {
    return false;
  }
  if (!bmiWrite8(REG_ACC_RANGE, 0x08)) {
    return false;  // +/- 8 g
  }
  if (!bmiWrite8(REG_GYR_CONF, 0x28)) {
    return false;
  }
  if (!bmiWrite8(REG_GYR_RANGE, 0x00)) {
    return false;  // +/- 2000 dps
  }
  delay(10);

  if (!bmiWrite8(REG_CMD, CMD_ACCEL_NORMAL)) {
    return false;
  }
  delay(50);
  if (!bmiWrite8(REG_CMD, CMD_GYRO_NORMAL)) {
    return false;
  }
  delay(100);

  uint8_t pmu = 0;
  if (bmiReadBytes(REG_PMU_STATUS, &pmu, 1)) {
    Serial.print("# PMU status 0x");
    Serial.println(pmu, HEX);
  }
  zeroStreak = 0;
  return true;
}

bool bmi160ReadRaw(int16_t &gx, int16_t &gy, int16_t &gz, int16_t &ax, int16_t &ay, int16_t &az) {
  bmi160EnsureSensorsOn();

  uint8_t buf[12];
  bool gotValid = false;

  for (int attempt = 0; attempt < 5; attempt++) {
    delay(12);
    if (!bmiReadBytes(REG_GYR_DATA, buf, 12)) {
      continue;
    }

    gx = (int16_t)((buf[1] << 8) | buf[0]);
    gy = (int16_t)((buf[3] << 8) | buf[2]);
    gz = (int16_t)((buf[5] << 8) | buf[4]);
    ax = (int16_t)((buf[7] << 8) | buf[6]);
    ay = (int16_t)((buf[9] << 8) | buf[8]);
    az = (int16_t)((buf[11] << 8) | buf[10]);

    if (accelHasGravity(ax, ay, az)) {
      gotValid = true;
      break;
    }
  }

  return gotValid;
}

void streamSample() {
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  if (!bmi160ReadRaw(gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw)) {
    zeroStreak++;
    readFailCount++;
    if (zeroStreak == 10) {
      Serial.println("# WARN BMI160 stale data — reinitializing sensor");
      bmi160Init();
      zeroStreak = 0;
    } else if (readFailCount == 1 || (readFailCount % 100) == 0) {
      Serial.print("# WARN BMI160 read invalid (");
      Serial.print(readFailCount);
      Serial.println(")");
    }
    return;
  }

  readFailCount = 0;
  zeroStreak = 0;

  Serial.print(rawAccelToMS2(axRaw), 4);
  Serial.print(',');
  Serial.print(rawAccelToMS2(ayRaw), 4);
  Serial.print(',');
  Serial.print(rawAccelToMS2(azRaw), 4);
  Serial.print(',');
  Serial.print(rawGyroToDPS(gxRaw), 4);
  Serial.print(',');
  Serial.print(rawGyroToDPS(gyRaw), 4);
  Serial.print(',');
  Serial.print(rawGyroToDPS(gzRaw), 4);
  Serial.print(",0,0,0");
  Serial.println();
}

void emitCalibrationJson() {
  StaticJsonDocument<768> doc;
  JsonObject gyro = doc.createNestedObject("gyro_offset");
  gyro["x"] = cal.gyro_offset[0];
  gyro["y"] = cal.gyro_offset[1];
  gyro["z"] = cal.gyro_offset[2];

  JsonObject accelOff = doc.createNestedObject("accel_offset");
  accelOff["x"] = cal.accel_offset[0];
  accelOff["y"] = cal.accel_offset[1];
  accelOff["z"] = cal.accel_offset[2];

  JsonObject accelScale = doc.createNestedObject("accel_scale");
  accelScale["x"] = cal.accel_scale[0];
  accelScale["y"] = cal.accel_scale[1];
  accelScale["z"] = cal.accel_scale[2];

  JsonObject magOff = doc.createNestedObject("mag_offset");
  magOff["x"] = cal.mag_offset[0];
  magOff["y"] = cal.mag_offset[1];
  magOff["z"] = cal.mag_offset[2];

  JsonArray soft = doc.createNestedArray("mag_soft_iron");
  for (int i = 0; i < 3; i++) {
    JsonArray row = soft.createNestedArray();
    for (int j = 0; j < 3; j++) {
      row.add(cal.mag_soft_iron[i][j]);
    }
  }

  Serial.print("CAL ");
  serializeJson(doc, Serial);
  Serial.println();
}

bool parseAndApplyCalibration(const char *json) {
  StaticJsonDocument<768> doc;
  if (deserializeJson(doc, json)) {
    return false;
  }

  JsonObject gyro = doc["gyro_offset"];
  JsonObject accelOff = doc["accel_offset"];
  JsonObject accelScale = doc["accel_scale"];
  JsonObject magOff = doc["mag_offset"];
  JsonArray soft = doc["mag_soft_iron"];

  if (gyro.isNull() || accelOff.isNull() || accelScale.isNull() || magOff.isNull() || soft.isNull()) {
    return false;
  }

  cal.gyro_offset[0] = gyro["x"] | 0.0f;
  cal.gyro_offset[1] = gyro["y"] | 0.0f;
  cal.gyro_offset[2] = gyro["z"] | 0.0f;

  cal.accel_offset[0] = accelOff["x"] | 0.0f;
  cal.accel_offset[1] = accelOff["y"] | 0.0f;
  cal.accel_offset[2] = accelOff["z"] | 0.0f;

  cal.accel_scale[0] = accelScale["x"] | 1.0f;
  cal.accel_scale[1] = accelScale["y"] | 1.0f;
  cal.accel_scale[2] = accelScale["z"] | 1.0f;

  cal.mag_offset[0] = magOff["x"] | 0.0f;
  cal.mag_offset[1] = magOff["y"] | 0.0f;
  cal.mag_offset[2] = magOff["z"] | 0.0f;

  for (int i = 0; i < 3; i++) {
    JsonArray row = soft[i];
    for (int j = 0; j < 3; j++) {
      cal.mag_soft_iron[i][j] = row[j] | ((i == j) ? 1.0f : 0.0f);
    }
  }

  saveCalibration();
  return true;
}

void handleSerialCommand(String &line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line == "PING") {
    Serial.println("PONG");
    return;
  }

  if (line == "CAL_MODE_ON") {
    calibrationMode = true;
    lastSampleMs = 0;
    readFailCount = 0;
    zeroStreak = 0;
    bmi160EnsureSensorsOn();
    Serial.println("CAL_MODE_ON");
    return;
  }

  if (line == "CAL_MODE_OFF") {
    calibrationMode = false;
    Serial.println("CAL_MODE_OFF");
    return;
  }

  if (line == "GET_CAL") {
    emitCalibrationJson();
    return;
  }

  if (line == "I2C_SCAN") {
    scanI2CBus();
    return;
  }

  if (line.startsWith("SET_CAL ")) {
    String json = line.substring(8);
    if (parseAndApplyCalibration(json.c_str())) {
      Serial.println("CAL_OK");
    } else {
      Serial.println("CAL_ERR");
    }
    return;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  initI2C();
  scanI2CBus();

  bmi160Addr = findBmi160Address();
  if (!bmi160Addr) {
    Serial.println("# ERR BMI160 not found at 0x68 or 0x69");
    Serial.println("# Check: power LED, SDA/SCL not swapped, common GND");
    Serial.println("# Some boards need 4.7k pull-ups on SDA/SCL if missing");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("# BMI160 found at 0x");
  Serial.println(bmi160Addr, HEX);

  if (!bmi160Init()) {
    Serial.println("# ERR BMI160 init failed");
    while (true) {
      delay(1000);
    }
  }

  {
    int16_t gx, gy, gz, ax, ay, az;
    delay(100);
    if (bmi160ReadRaw(gx, gy, gz, ax, ay, az)) {
      Serial.print("# Test read  ax=");
      Serial.print(ax);
      Serial.print(" ay=");
      Serial.print(ay);
      Serial.print(" az=");
      Serial.print(az);
      Serial.print("  gx=");
      Serial.print(gx);
      Serial.print(" gy=");
      Serial.print(gy);
      Serial.print(" gz=");
      Serial.println(gz);
      if (ax == 0 && ay == 0 && az == 0) {
        Serial.println("# WARN accel all zero — place board flat, then reset");
      }
    } else {
      Serial.println("# WARN test read failed");
    }
  }

  loadCalibration();
  Serial.println("# READY BMI160 streamer idle; send CAL_MODE_ON to stream");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleSerialCommand(line);
  }

  uint32_t now = millis();
  if (calibrationMode && now - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs = now;
    streamSample();
  }
}
