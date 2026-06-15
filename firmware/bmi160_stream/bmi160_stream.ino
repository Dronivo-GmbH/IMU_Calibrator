/*
 * BMI160 (+ optional BMM150) serial streamer for IMU calibration GUI
 *
 * Hardware: Arduino Uno/Nano/Mega or ESP32 + BMI160 I2C breakout
 * Optional BMM150 on auxiliary port (SparkFun 9-DoF) or separate I2C 0x10
 *
 * Required libraries (Arduino Library Manager):
 *   - SparkFun BMI160 Arduino Library
 *   - ArduinoJson (by Benoit Blanchon) v6.x
 *
 * Wiring (I2C):
 *   SDA -> A4 (Uno) / GPIO21 (ESP32)
 *   SCL -> A5 (Uno) / GPIO22 (ESP32)
 *
 * Serial: 115200 baud
 *
 * Protocol:
 *   Stream:  ax,ay,az,gx,gy,gz,mx,my,mz
 *   Commands:
 *     PING
 *     CAL_MODE_ON
 *     CAL_MODE_OFF
 *     GET_CAL
 *     SET_CAL {"gyro_offset":{...}, ...}
 *   Responses:
 *     PONG / CAL_MODE_ON / CAL_MODE_OFF / CAL_OK / CAL_ERR / CAL {...json...}
 */

#include <Wire.h>
#include <ArduinoJson.h>

#if defined(ESP32)
#include <Preferences.h>
#else
#include <EEPROM.h>
#endif

#include "SparkFunBMI160.h"
#include "SparkFunBMM150Aux.h"

// ---------- Config ----------
static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t SAMPLE_HZ = 100;
static const uint32_t SAMPLE_MS = 1000 / SAMPLE_HZ;

static const float ACCEL_RANGE_G = 8.0f;
static const float GYRO_RANGE_DPS = 2000.0f;
static const float GRAVITY = 9.80665f;

static const uint32_t CAL_MAGIC = 0xCA1B1601;

// ---------- Sensors ----------
SparkFunBMI160 imu;
SparkFunBMM150Aux mag;
bool magReady = false;
bool calibrationMode = false;

// ---------- Calibration ----------
struct CalProfile {
  uint32_t magic;
  float gyro_offset[3];
  float accel_offset[3];
  float accel_scale[3];
  float mag_offset[3];
  float mag_soft_iron[3][3];
};

CalProfile cal;

// ---------- Timing ----------
uint32_t lastSampleMs = 0;

// ---------- Forward declarations ----------
void loadCalibration();
void saveCalibration();
void resetCalibrationDefaults();
void emitCalibrationJson();
bool parseAndApplyCalibration(const char *json);
void handleSerialCommand(String &line);
void streamSample();
float rawAccelToMS2(int16_t raw);
float rawGyroToDPS(int16_t raw);

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

void applyMagCorrection(float mx, float my, float mz, float &ox, float &oy, float &oz) {
  float vx = mx - cal.mag_offset[0];
  float vy = my - cal.mag_offset[1];
  float vz = mz - cal.mag_offset[2];
  ox = cal.mag_soft_iron[0][0] * vx + cal.mag_soft_iron[0][1] * vy + cal.mag_soft_iron[0][2] * vz;
  oy = cal.mag_soft_iron[1][0] * vx + cal.mag_soft_iron[1][1] * vy + cal.mag_soft_iron[1][2] * vz;
  oz = cal.mag_soft_iron[2][0] * vx + cal.mag_soft_iron[2][1] * vy + cal.mag_soft_iron[2][2] * vz;
}

void streamSample() {
  int16_t axRaw, ayRaw, azRaw;
  int16_t gxRaw, gyRaw, gzRaw;
  float mxRaw = 0.0f, myRaw = 0.0f, mzRaw = 0.0f;

  axRaw = imu.getRawAccelX();
  ayRaw = imu.getRawAccelY();
  azRaw = imu.getRawAccelZ();
  gxRaw = imu.getRawGyroX();
  gyRaw = imu.getRawGyroY();
  gzRaw = imu.getRawGyroZ();

  if (magReady) {
    int16_t mxInt, myInt, mzInt;
    if (mag.readMag(mxInt, myInt, mzInt)) {
      // BMM150 aux output is typically in 0.3 µT/LSB for default settings
      mxRaw = mxInt * 0.3f;
      myRaw = myInt * 0.3f;
      mzRaw = mzInt * 0.3f;
    }
  }

  float ax = rawAccelToMS2(axRaw);
  float ay = rawAccelToMS2(ayRaw);
  float az = rawAccelToMS2(azRaw);
  float gx = rawGyroToDPS(gxRaw);
  float gy = rawGyroToDPS(gyRaw);
  float gz = rawGyroToDPS(gzRaw);

  // Stream raw values — GUI applies calibration
  Serial.print(ax, 4);
  Serial.print(',');
  Serial.print(ay, 4);
  Serial.print(',');
  Serial.print(az, 4);
  Serial.print(',');
  Serial.print(gx, 4);
  Serial.print(',');
  Serial.print(gy, 4);
  Serial.print(',');
  Serial.print(gz, 4);
  Serial.print(',');
  Serial.print(mxRaw, 4);
  Serial.print(',');
  Serial.print(myRaw, 4);
  Serial.print(',');
  Serial.println(mzRaw, 4);
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

  Wire.begin();
  Wire.setClock(400000);

  if (imu.begin(BMI160SlaveSelect::BMI160_I2C_ADDR) != BMI160_OK) {
    Serial.println("# ERR BMI160 init failed");
    while (true) {
      delay(1000);
    }
  }

  imu.setAccelRange(BMI160_ACCEL_RANGE_8G);
  imu.setGyroRange(BMI160_GYRO_RANGE_2000_DPS);

  if (mag.begin(BMI160SlaveSelect::BMI160_I2C_ADDR, imu) == BMM150_OK) {
    magReady = true;
  } else {
    magReady = false;
    Serial.println("# WARN BMM150 not found — mag values will be zero");
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
