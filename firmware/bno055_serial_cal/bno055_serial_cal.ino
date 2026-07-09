/*
 * BNO055 serial calibration bridge for Arduino Nano / ESP32.
 *
 * Hardware path:
 *   Laptop GUI <-> USB serial <-> MCU <-> I2C <-> BNO055
 *
 * Serial commands:
 *   PING
 *   CAL_MODE_ON
 *   CAL_MODE_OFF
 *   BNO_BEGIN
 *   BNO_GET_STATUS
 *   BNO_READ_PROFILE
 *   BNO_WRITE_PROFILE {"calibration_bytes":[...22...],"axis_map_config":36,"axis_map_sign":0,"address":40}
 *   BNO_READ_EULER
 *   BNO_READ_QUAT
 *
 * Serial responses:
 *   PONG
 *   CAL_MODE_ON / CAL_MODE_OFF
 *   BNO_BEGIN {...json...}
 *   BNO_STATUS {...json...}
 *   BNO_PROFILE {...json...}
 *   BNO_EULER {...json...}
 *   BNO_QUAT {...json...}
 *   BNO_WRITE_OK / BNO_WRITE_ERR
 *   BNO_ERR {...json...}
 *
 * Streaming format while CAL_MODE_ON:
 *   ax,ay,az,gx,gy,gz,mx,my,mz   (physical units: m/s^2, deg/s, uT)
 */

#include <Wire.h>
#include <ArduinoJson.h>

#if defined(ESP32)
#include <Preferences.h>
#else
#include <EEPROM.h>
#endif

static const uint32_t SERIAL_BAUD = 115200;
static const uint8_t BNO055_ADDR_A = 0x28;
static const uint8_t BNO055_ADDR_B = 0x29;
static const uint8_t BNO055_CHIP_ID = 0xA0;
static const uint32_t POWER_UP_DELAY_MS = 650;
static const uint32_t CONFIG_MODE_DELAY_MS = 19;
static const uint32_t NDOF_DELAY_MS = 7;
static const uint32_t SAMPLE_HZ = 50;
static const uint32_t SAMPLE_MS = 1000 / SAMPLE_HZ;
static const float GRAVITY = 9.80665f;
static const uint32_t PROFILE_MAGIC = 0xB0055A11;

enum BNO055Registers : uint8_t {
  REG_CHIP_ID = 0x00,
  REG_PAGE_ID = 0x07,
  REG_EULER_H_LSB = 0x1A,
  REG_QUAT_W_LSB = 0x20,
  REG_CALIB_STAT = 0x35,
  REG_UNIT_SEL = 0x3B,
  REG_OPR_MODE = 0x3D,
  REG_PWR_MODE = 0x3E,
  REG_SYS_STATUS = 0x39,
  REG_SYS_ERR = 0x3A,
  REG_AXIS_MAP_CONFIG = 0x41,
  REG_AXIS_MAP_SIGN = 0x42,
  REG_ACCEL_DATA_X_LSB = 0x08,
  REG_MAG_DATA_X_LSB = 0x0E,
  REG_GYRO_DATA_X_LSB = 0x14,
  REG_CAL_PROFILE_START = 0x55
};

enum BNO055Mode : uint8_t {
  MODE_CONFIG = 0x00,
  MODE_NDOF = 0x0C
};

struct BNO055CalibrationStatus {
  uint8_t sys;
  uint8_t gyr;
  uint8_t acc;
  uint8_t mag;
  uint8_t raw;

  bool fullyCalibrated() const {
    return sys == 3 && gyr == 3 && acc == 3 && mag == 3;
  }
};

struct BNO055Profile {
  uint8_t bytes[22];
  uint8_t axisMapConfig;
  uint8_t axisMapSign;
  uint8_t address;
};

struct StoredProfile {
  uint32_t magic;
  BNO055Profile profile;
};

struct EulerData {
  float heading;
  float roll;
  float pitch;
};

struct QuaternionData {
  float w;
  float x;
  float y;
  float z;
};

class BNO055Driver {
 public:
  BNO055Driver() : address_(0), axisMapConfig_(0x24), axisMapSign_(0x00), currentMode_(MODE_CONFIG), began_(false) {}

  bool begin(uint8_t preferredAddress = 0, uint8_t axisMapConfig = 0x24, uint8_t axisMapSign = 0x00) {
    axisMapConfig_ = axisMapConfig;
    axisMapSign_ = axisMapSign;
    delay(POWER_UP_DELAY_MS);

    uint8_t found = 0;
    if (preferredAddress == BNO055_ADDR_A || preferredAddress == BNO055_ADDR_B) {
      if (readU8(preferredAddress, REG_CHIP_ID) == BNO055_CHIP_ID) {
        found = preferredAddress;
      }
    } else {
      if (readU8(BNO055_ADDR_A, REG_CHIP_ID) == BNO055_CHIP_ID) {
        found = BNO055_ADDR_A;
      } else if (readU8(BNO055_ADDR_B, REG_CHIP_ID) == BNO055_CHIP_ID) {
        found = BNO055_ADDR_B;
      }
    }

    if (found == 0) {
      setError(100, "BNO055 CHIP_ID 0xA0 not found on 0x28 or 0x29");
      return false;
    }

    address_ = found;
    if (!writeU8(REG_PAGE_ID, 0x00)) return false;
    if (!set_mode(MODE_CONFIG)) return false;
    if (!writeU8(REG_PWR_MODE, 0x00)) return false;
    if (!writeU8(REG_UNIT_SEL, 0x80)) return false;
    if (!writeU8(REG_AXIS_MAP_CONFIG, axisMapConfig_)) return false;
    if (!writeU8(REG_AXIS_MAP_SIGN, axisMapSign_)) return false;
    if (!set_mode(MODE_NDOF)) return false;
    if (!checkSystemStatus()) return false;
    began_ = true;
    clearError();
    return true;
  }

  bool set_mode(uint8_t mode) {
    if (address_ == 0) {
      setError(101, "Sensor address not initialized");
      return false;
    }
    if (!writeU8(REG_PAGE_ID, 0x00)) return false;
    if (!writeU8(REG_OPR_MODE, mode)) return false;
    currentMode_ = mode;
    delay(mode == MODE_CONFIG ? CONFIG_MODE_DELAY_MS : NDOF_DELAY_MS);
    return true;
  }

  BNO055CalibrationStatus get_calibration_status() {
    BNO055CalibrationStatus status = {0, 0, 0, 0, 0};
    status.raw = readU8(address_, REG_CALIB_STAT);
    status.sys = (status.raw >> 6) & 0x03;
    status.gyr = (status.raw >> 4) & 0x03;
    status.acc = (status.raw >> 2) & 0x03;
    status.mag = status.raw & 0x03;
    return status;
  }

  bool wait_for_full_calibration(unsigned long timeoutMs, Stream &out) {
    out.println(F("BNO_CAL Gyroscope: keep device completely still until GYR reaches 3."));
    out.println(F("BNO_CAL Accelerometer: place the device in 6 stable orientations."));
    out.println(F("BNO_CAL Magnetometer: move in random 3D / figure-eight motion away from interference."));
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
      if (!checkSystemStatus()) return false;
      BNO055CalibrationStatus status = get_calibration_status();
      emitStatus(out, status);
      if (status.fullyCalibrated()) return true;
      delay(500);
    }
    setError(102, "Timed out waiting for full calibration");
    return false;
  }

  bool read_calibration_profile(BNO055Profile &profile) {
    BNO055CalibrationStatus status = get_calibration_status();
    if (!status.fullyCalibrated()) {
      setError(103, "Offsets/radius can only be read after full calibration");
      return false;
    }

    uint8_t previousMode = currentMode_;
    if (!set_mode(MODE_CONFIG)) return false;
    if (!readBlock(REG_CAL_PROFILE_START, profile.bytes, 22)) {
      if (previousMode != MODE_CONFIG) set_mode(previousMode);
      return false;
    }
    profile.axisMapConfig = axisMapConfig_;
    profile.axisMapSign = axisMapSign_;
    profile.address = address_;
    if (previousMode != MODE_CONFIG && !set_mode(previousMode)) return false;
    return true;
  }

  bool write_calibration_profile(const BNO055Profile &profile) {
    uint8_t previousMode = currentMode_;
    if (!set_mode(MODE_CONFIG)) return false;
    axisMapConfig_ = profile.axisMapConfig;
    axisMapSign_ = profile.axisMapSign;
    if (!writeU8(REG_AXIS_MAP_CONFIG, axisMapConfig_)) {
      if (previousMode != MODE_CONFIG) set_mode(previousMode);
      return false;
    }
    if (!writeU8(REG_AXIS_MAP_SIGN, axisMapSign_)) {
      if (previousMode != MODE_CONFIG) set_mode(previousMode);
      return false;
    }
    if (!writeBlock(REG_CAL_PROFILE_START, profile.bytes, 22)) {
      if (previousMode != MODE_CONFIG) set_mode(previousMode);
      return false;
    }
    if (!set_mode(MODE_NDOF)) return false;
    if (previousMode != MODE_NDOF && previousMode != MODE_CONFIG) {
      if (!set_mode(previousMode)) return false;
    }
    return checkSystemStatus();
  }

  bool save_profile() {
    BNO055Profile profile;
    if (!read_calibration_profile(profile)) return false;
    StoredProfile stored;
    stored.magic = PROFILE_MAGIC;
    stored.profile = profile;
#if defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("bno055", false)) {
      setError(104, "Preferences open failed");
      return false;
    }
    prefs.putBytes("profile", &stored, sizeof(stored));
    prefs.end();
#else
    EEPROM.put(0, stored);
#endif
    return true;
  }

  bool load_profile() {
    StoredProfile stored;
#if defined(ESP32)
    Preferences prefs;
    if (!prefs.begin("bno055", true)) {
      setError(105, "Preferences open failed");
      return false;
    }
    prefs.getBytes("profile", &stored, sizeof(stored));
    prefs.end();
#else
    EEPROM.get(0, stored);
#endif
    if (stored.magic != PROFILE_MAGIC) {
      setError(106, "No saved BNO055 profile found in nonvolatile storage");
      return false;
    }
    return write_calibration_profile(stored.profile);
  }

  bool read_euler(EulerData &euler) {
    uint8_t buffer[6];
    if (!readBlock(REG_EULER_H_LSB, buffer, 6)) return false;
    euler.heading = int16From(buffer[0], buffer[1]) / 16.0f;
    euler.roll = int16From(buffer[2], buffer[3]) / 16.0f;
    euler.pitch = int16From(buffer[4], buffer[5]) / 16.0f;
    return true;
  }

  bool read_quaternion(QuaternionData &quat) {
    uint8_t buffer[8];
    if (!readBlock(REG_QUAT_W_LSB, buffer, 8)) return false;
    const float scale = 1.0f / 16384.0f;
    quat.w = int16From(buffer[0], buffer[1]) * scale;
    quat.x = int16From(buffer[2], buffer[3]) * scale;
    quat.y = int16From(buffer[4], buffer[5]) * scale;
    quat.z = int16From(buffer[6], buffer[7]) * scale;
    return true;
  }

  bool readImuSample(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz) {
    uint8_t accelBuf[6];
    uint8_t gyroBuf[6];
    uint8_t magBuf[6];
    if (!readBlock(REG_ACCEL_DATA_X_LSB, accelBuf, 6)) return false;
    if (!readBlock(REG_GYRO_DATA_X_LSB, gyroBuf, 6)) return false;
    if (!readBlock(REG_MAG_DATA_X_LSB, magBuf, 6)) return false;

    ax = int16From(accelBuf[0], accelBuf[1]) / 100.0f;
    ay = int16From(accelBuf[2], accelBuf[3]) / 100.0f;
    az = int16From(accelBuf[4], accelBuf[5]) / 100.0f;
    gx = int16From(gyroBuf[0], gyroBuf[1]) / 16.0f;
    gy = int16From(gyroBuf[2], gyroBuf[3]) / 16.0f;
    gz = int16From(gyroBuf[4], gyroBuf[5]) / 16.0f;
    mx = int16From(magBuf[0], magBuf[1]) / 16.0f;
    my = int16From(magBuf[2], magBuf[3]) / 16.0f;
    mz = int16From(magBuf[4], magBuf[5]) / 16.0f;
    return true;
  }

  bool checkSystemStatus() {
    uint8_t status = readU8(address_, REG_SYS_STATUS);
    if (status == 1) {
      uint8_t err = readU8(address_, REG_SYS_ERR);
      setError(err, sysErrMessage(err));
      return false;
    }
    clearError();
    return true;
  }

  uint8_t address() const { return address_; }
  uint8_t axisMapConfig() const { return axisMapConfig_; }
  uint8_t axisMapSign() const { return axisMapSign_; }
  bool began() const { return began_; }
  int lastErrorCode() const { return lastErrorCode_; }
  const String &lastErrorMessage() const { return lastErrorMessage_; }

 private:
  uint8_t address_;
  uint8_t axisMapConfig_;
  uint8_t axisMapSign_;
  uint8_t currentMode_;
  bool began_;
  int lastErrorCode_ = 0;
  String lastErrorMessage_;

  void setError(int code, const String &message) {
    lastErrorCode_ = code;
    lastErrorMessage_ = message;
  }

  void clearError() {
    lastErrorCode_ = 0;
    lastErrorMessage_ = "";
  }

  bool writeU8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission() != 0) {
      setError(107, "I2C write failed");
      return false;
    }
    return true;
  }

  uint8_t readU8(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      return 0xFF;
    }
    if (Wire.requestFrom((int)addr, 1) != 1) {
      return 0xFF;
    }
    return Wire.read();
  }

  bool readBlock(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      setError(108, "I2C burst read address phase failed");
      return false;
    }
    uint8_t received = Wire.requestFrom((int)address_, (int)length);
    if (received != length) {
      setError(109, "I2C burst read length mismatch");
      return false;
    }
    for (uint8_t i = 0; i < length; ++i) {
      buffer[i] = Wire.read();
    }
    return true;
  }

  bool writeBlock(uint8_t reg, const uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    for (uint8_t i = 0; i < length; ++i) {
      Wire.write(buffer[i]);
    }
    if (Wire.endTransmission() != 0) {
      setError(110, "I2C burst write failed");
      return false;
    }
    return true;
  }

  static int16_t int16From(uint8_t lsb, uint8_t msb) {
    return (int16_t)((msb << 8) | lsb);
  }

  static String sysErrMessage(uint8_t code) {
    switch (code) {
      case 0: return "no error";
      case 1: return "peripheral initialization error";
      case 2: return "system initialization error";
      case 3: return "self-test failed";
      case 4: return "register map value out of range";
      case 5: return "register map address out of range";
      case 6: return "register map write error";
      case 7: return "low power mode not available";
      case 8: return "accelerometer power mode not available";
      case 9: return "fusion algorithm configuration error";
      case 10: return "sensor configuration error";
      default: return "unknown error";
    }
  }

  static void emitStatus(Stream &out, const BNO055CalibrationStatus &status) {
    StaticJsonDocument<96> doc;
    doc["sys"] = status.sys;
    doc["gyr"] = status.gyr;
    doc["acc"] = status.acc;
    doc["mag"] = status.mag;
    doc["raw"] = status.raw;
    out.print(F("BNO_STATUS "));
    serializeJson(doc, out);
    out.println();
  }
};

BNO055Driver bno;
bool calibrationMode = false;
unsigned long lastSampleMs = 0;
String commandBuffer;

void emitError(const String &message, int code) {
  StaticJsonDocument<128> doc;
  doc["code"] = code;
  doc["message"] = message;
  Serial.print(F("BNO_ERR "));
  serializeJson(doc, Serial);
  Serial.println();
}

void emitBegin() {
  StaticJsonDocument<96> doc;
  doc["address"] = bno.address();
  doc["axis_map_config"] = bno.axisMapConfig();
  doc["axis_map_sign"] = bno.axisMapSign();
  doc["mode"] = "NDOF";
  Serial.print(F("BNO_BEGIN "));
  serializeJson(doc, Serial);
  Serial.println();
}

void emitStatus() {
  BNO055CalibrationStatus status = bno.get_calibration_status();
  StaticJsonDocument<96> doc;
  doc["sys"] = status.sys;
  doc["gyr"] = status.gyr;
  doc["acc"] = status.acc;
  doc["mag"] = status.mag;
  doc["raw"] = status.raw;
  Serial.print(F("BNO_STATUS "));
  serializeJson(doc, Serial);
  Serial.println();
}

void emitProfile(const BNO055Profile &profile) {
  StaticJsonDocument<256> doc;
  doc["axis_map_config"] = profile.axisMapConfig;
  doc["axis_map_sign"] = profile.axisMapSign;
  doc["address"] = profile.address;
  JsonArray arr = doc.createNestedArray("calibration_bytes");
  for (uint8_t i = 0; i < 22; ++i) {
    arr.add(profile.bytes[i]);
  }
  Serial.print(F("BNO_PROFILE "));
  serializeJson(doc, Serial);
  Serial.println();
}

void emitEuler() {
  EulerData euler;
  if (!bno.read_euler(euler)) {
    emitError(bno.lastErrorMessage(), bno.lastErrorCode());
    return;
  }
  StaticJsonDocument<128> doc;
  doc["heading"] = euler.heading;
  doc["roll"] = euler.roll;
  doc["pitch"] = euler.pitch;
  Serial.print(F("BNO_EULER "));
  serializeJson(doc, Serial);
  Serial.println();
}

void emitQuaternion() {
  QuaternionData quat;
  if (!bno.read_quaternion(quat)) {
    emitError(bno.lastErrorMessage(), bno.lastErrorCode());
    return;
  }
  StaticJsonDocument<160> doc;
  doc["w"] = quat.w;
  doc["x"] = quat.x;
  doc["y"] = quat.y;
  doc["z"] = quat.z;
  Serial.print(F("BNO_QUAT "));
  serializeJson(doc, Serial);
  Serial.println();
}

bool parseProfileJson(const String &json, BNO055Profile &profile) {
  StaticJsonDocument<320> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    emitError(String(F("Invalid profile JSON: ")) + err.c_str(), 111);
    return false;
  }
  JsonArray arr = doc["calibration_bytes"];
  if (arr.isNull() || arr.size() != 22) {
    emitError("Profile must contain 22 calibration_bytes", 112);
    return false;
  }
  for (uint8_t i = 0; i < 22; ++i) {
    profile.bytes[i] = arr[i] | 0;
  }
  profile.axisMapConfig = doc["axis_map_config"] | 0x24;
  profile.axisMapSign = doc["axis_map_sign"] | 0x00;
  profile.address = doc["address"] | bno.address();
  return true;
}

void streamSample() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  if (!bno.readImuSample(ax, ay, az, gx, gy, gz, mx, my, mz)) {
    emitError(bno.lastErrorMessage(), bno.lastErrorCode());
    return;
  }
  Serial.print(ax, 4); Serial.print(',');
  Serial.print(ay, 4); Serial.print(',');
  Serial.print(az, 4); Serial.print(',');
  Serial.print(gx, 4); Serial.print(',');
  Serial.print(gy, 4); Serial.print(',');
  Serial.print(gz, 4); Serial.print(',');
  Serial.print(mx, 4); Serial.print(',');
  Serial.print(my, 4); Serial.print(',');
  Serial.println(mz, 4);
}

void handleSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "PING") {
    Serial.println(F("PONG"));
    return;
  }

  if (line == "CAL_MODE_ON") {
    calibrationMode = true;
    lastSampleMs = 0;
    Serial.println(F("CAL_MODE_ON"));
    return;
  }

  if (line == "CAL_MODE_OFF") {
    calibrationMode = false;
    Serial.println(F("CAL_MODE_OFF"));
    return;
  }

  if (line == "BNO_BEGIN") {
    if (!bno.begin()) {
      emitError(bno.lastErrorMessage(), bno.lastErrorCode());
      return;
    }
    emitBegin();
    return;
  }

  if (!bno.began()) {
    emitError("BNO055 not initialized. Send BNO_BEGIN first.", 113);
    return;
  }

  if (line == "BNO_GET_STATUS") {
    if (!bno.checkSystemStatus()) {
      emitError(bno.lastErrorMessage(), bno.lastErrorCode());
      return;
    }
    emitStatus();
    return;
  }

  if (line == "BNO_READ_PROFILE") {
    BNO055Profile profile;
    if (!bno.read_calibration_profile(profile)) {
      emitError(bno.lastErrorMessage(), bno.lastErrorCode());
      return;
    }
    emitProfile(profile);
    return;
  }

  if (line.startsWith("BNO_WRITE_PROFILE ")) {
    BNO055Profile profile;
    if (!parseProfileJson(line.substring(18), profile)) return;
    if (!bno.write_calibration_profile(profile)) {
      emitError(bno.lastErrorMessage(), bno.lastErrorCode());
      Serial.println(F("BNO_WRITE_ERR"));
      return;
    }
    Serial.println(F("BNO_WRITE_OK"));
    return;
  }

  if (line == "BNO_READ_EULER") {
    emitEuler();
    return;
  }

  if (line == "BNO_READ_QUAT") {
    emitQuaternion();
    return;
  }

  if (line == "BNO_SAVE_PROFILE") {
    if (!bno.save_profile()) {
      emitError(bno.lastErrorMessage(), bno.lastErrorCode());
      return;
    }
    Serial.println(F("BNO_WRITE_OK"));
    return;
  }

  if (line == "BNO_LOAD_PROFILE") {
    if (!bno.load_profile()) {
      emitError(bno.lastErrorMessage(), bno.lastErrorCode());
      return;
    }
    Serial.println(F("BNO_WRITE_OK"));
    return;
  }

  emitError(String(F("Unknown command: ")) + line, 114);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) {
    delay(10);
  }
  Wire.begin();
  Wire.setClock(100000);
  Serial.println(F("# READY BNO055 serial calibrator; send BNO_BEGIN"));
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (commandBuffer.length() > 0) {
        handleSerialCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      commandBuffer += c;
      if (commandBuffer.length() > 300) {
        commandBuffer = "";
      }
    }
  }

  if (calibrationMode && bno.began()) {
    unsigned long now = millis();
    if (now - lastSampleMs >= SAMPLE_MS) {
      lastSampleMs = now;
      streamSample();
    }
  }
}
