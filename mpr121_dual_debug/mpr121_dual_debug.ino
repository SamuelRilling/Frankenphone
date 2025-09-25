#include <Arduino.h>
#include <Wire.h>

// I2C bus assignments (ESP32):
// Board 1 @ 0x5A on SDA=13, SCL=14
// Board 2 @ 0x5B on SDA=27, SCL=26

static const uint8_t MPR121_ADDR_BOARD1 = 0x5A;
static const uint8_t MPR121_ADDR_BOARD2 = 0x5B;

static const int BOARD1_SDA_PIN = 13;
static const int BOARD1_SCL_PIN = 14;
static const int BOARD2_SDA_PIN = 27;
static const int BOARD2_SCL_PIN = 26;

// ESP32 supports two I2C controllers. Create distinct buses for each board.
TwoWire i2cBoard1 = TwoWire(0);
TwoWire i2cBoard2 = TwoWire(1);

// MPR121 register addresses
static const uint8_t MPR121_TOUCHSTATUS_L = 0x00;
static const uint8_t MPR121_TOUCHSTATUS_H = 0x01;
static const uint8_t MPR121_ECR           = 0x5E; // Electrode Configuration
static const uint8_t MPR121_SOFTRESET     = 0x80; // Write 0x63 to reset

// Filter and global CDC/CDT registers (minimal config values)
static const uint8_t MHD_R = 0x2B;
static const uint8_t NHD_R = 0x2C;
static const uint8_t NCL_R = 0x2D;
static const uint8_t FDL_R = 0x2E;
static const uint8_t MHD_F = 0x2F;
static const uint8_t NHD_F = 0x30;
static const uint8_t NCL_F = 0x31;
static const uint8_t FDL_F = 0x32;

static const uint8_t ELE0_T = 0x41; // Touch threshold for E0
static const uint8_t ELE0_R = 0x42; // Release threshold for E0

// Write a single byte to a register; returns true on ACK
static bool writeRegister(TwoWire &wire, uint8_t deviceAddress, uint8_t reg, uint8_t value) {
  wire.beginTransmission(deviceAddress);
  wire.write(reg);
  wire.write(value);
  return wire.endTransmission() == 0;
}

// Read a sequence of bytes starting at a register; returns true on success
static bool readRegisters(TwoWire &wire, uint8_t deviceAddress, uint8_t startReg, uint8_t *buffer, size_t length) {
  wire.beginTransmission(deviceAddress);
  wire.write(startReg);
  if (wire.endTransmission(false) != 0) { // restart condition
    return false;
  }
  size_t received = wire.requestFrom(static_cast<int>(deviceAddress), static_cast<int>(length));
  if (received != length) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    buffer[i] = wire.read();
  }
  return true;
}

// Perform a soft reset
static bool mpr121SoftReset(TwoWire &wire, uint8_t deviceAddress) {
  bool ok = writeRegister(wire, deviceAddress, MPR121_SOFTRESET, 0x63);
  delay(1);
  return ok;
}

// Put device in stop mode (ECR = 0x00)
static bool mpr121Stop(TwoWire &wire, uint8_t deviceAddress) {
  return writeRegister(wire, deviceAddress, MPR121_ECR, 0x00);
}

// Minimal configuration for filters and thresholds
static bool mpr121Configure(TwoWire &wire, uint8_t deviceAddress) {
  bool ok = true;

  // Baseline filter configuration (values commonly used; minimal and robust)
  ok &= writeRegister(wire, deviceAddress, MHD_R, 0x01);
  ok &= writeRegister(wire, deviceAddress, NHD_R, 0x01);
  ok &= writeRegister(wire, deviceAddress, NCL_R, 0x00);
  ok &= writeRegister(wire, deviceAddress, FDL_R, 0x00);

  ok &= writeRegister(wire, deviceAddress, MHD_F, 0x01);
  ok &= writeRegister(wire, deviceAddress, NHD_F, 0x01);
  ok &= writeRegister(wire, deviceAddress, NCL_F, 0xFF);
  ok &= writeRegister(wire, deviceAddress, FDL_F, 0x02);

  // Set per-electrode touch/release thresholds (same for all 12 electrodes)
  const uint8_t touchThreshold = 12; // adjust as needed
  const uint8_t releaseThreshold = 6; // adjust as needed
  for (uint8_t i = 0; i < 12; ++i) {
    ok &= writeRegister(wire, deviceAddress, static_cast<uint8_t>(ELE0_T + i * 2), touchThreshold);
    ok &= writeRegister(wire, deviceAddress, static_cast<uint8_t>(ELE0_R + i * 2), releaseThreshold);
  }

  return ok;
}

// Enable electrodes and run (0x8F: baseline tracking enabled, 12 electrodes)
static bool mpr121Run(TwoWire &wire, uint8_t deviceAddress) {
  return writeRegister(wire, deviceAddress, MPR121_ECR, 0x8F);
}

// Initialize one MPR121 on the given I2C bus and address; prints debug info
static bool initMpr121(TwoWire &wire, uint8_t deviceAddress, const char *label) {
  Serial.print("[" ); Serial.print(label); Serial.println("] Init start");

  bool ok = true;
  ok &= mpr121SoftReset(wire, deviceAddress);
  Serial.print("[" ); Serial.print(label); Serial.print("] Soft reset: "); Serial.println(ok ? "OK" : "FAIL");
  if (!ok) return false;

  ok &= mpr121Stop(wire, deviceAddress);
  Serial.print("[" ); Serial.print(label); Serial.print("] Stop mode: "); Serial.println(ok ? "OK" : "FAIL");
  if (!ok) return false;

  ok &= mpr121Configure(wire, deviceAddress);
  Serial.print("[" ); Serial.print(label); Serial.print("] Configure: "); Serial.println(ok ? "OK" : "FAIL");
  if (!ok) return false;

  ok &= mpr121Run(wire, deviceAddress);
  Serial.print("[" ); Serial.print(label); Serial.print("] Run mode: "); Serial.println(ok ? "OK" : "FAIL");

  Serial.print("[" ); Serial.print(label); Serial.println("] Init done");
  return ok;
}

// Read current 12-bit touch status (lower 12 bits of two bytes)
static bool readTouchStatus(TwoWire &wire, uint8_t deviceAddress, uint16_t &statusOut) {
  uint8_t buf[2] = {0, 0};
  if (!readRegisters(wire, deviceAddress, MPR121_TOUCHSTATUS_L, buf, 2)) {
    return false;
  }
  statusOut = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
  statusOut &= 0x0FFF; // 12 electrodes
  return true;
}

static uint16_t previousStatusBoard1 = 0;
static uint16_t previousStatusBoard2 = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for serial */ }
  delay(100);

  Serial.println();
  Serial.println("MPR121 Dual-Board Debug (I2C on separate buses)");

  // Initialize I2C buses
  bool bus1Begun = i2cBoard1.begin(BOARD1_SDA_PIN, BOARD1_SCL_PIN, 100000); // 100kHz is fine
  bool bus2Begun = i2cBoard2.begin(BOARD2_SDA_PIN, BOARD2_SCL_PIN, 100000);

  Serial.print("[Board1 Bus] begin(" ); Serial.print(BOARD1_SDA_PIN); Serial.print(", "); Serial.print(BOARD1_SCL_PIN); Serial.print(") => "); Serial.println(bus1Begun ? "OK" : "FAIL");
  Serial.print("[Board2 Bus] begin(" ); Serial.print(BOARD2_SDA_PIN); Serial.print(", "); Serial.print(BOARD2_SCL_PIN); Serial.print(") => "); Serial.println(bus2Begun ? "OK" : "FAIL");

  if (!bus1Begun || !bus2Begun) {
    Serial.println("ERROR: Failed to start one or both I2C buses. Check wiring and pins.");
  }

  // Try to init both boards; proceed even if one fails to allow targeted debugging
  bool b1 = initMpr121(i2cBoard1, MPR121_ADDR_BOARD1, "Board1 0x5A");
  bool b2 = initMpr121(i2cBoard2, MPR121_ADDR_BOARD2, "Board2 0x5B");

  if (!b1) Serial.println("WARNING: Board 1 (0x5A) init failed.");
  if (!b2) Serial.println("WARNING: Board 2 (0x5B) init failed.");

  Serial.println("Setup complete. Touch any electrode to see messages.");
}

void loop() {
  uint16_t status1 = 0;
  uint16_t status2 = 0;

  bool ok1 = readTouchStatus(i2cBoard1, MPR121_ADDR_BOARD1, status1);
  bool ok2 = readTouchStatus(i2cBoard2, MPR121_ADDR_BOARD2, status2);

  if (!ok1) {
    static uint32_t lastErr1 = 0; // rate-limit errors
    if (millis() - lastErr1 > 1000) {
      Serial.println("ERROR: Failed to read touch status from Board 1 (0x5A)");
      lastErr1 = millis();
    }
  }
  if (!ok2) {
    static uint32_t lastErr2 = 0; // rate-limit errors
    if (millis() - lastErr2 > 1000) {
      Serial.println("ERROR: Failed to read touch status from Board 2 (0x5B)");
      lastErr2 = millis();
    }
  }

  // Report a simple message when any electrode becomes touched
  if (ok1) {
    uint16_t newTouches1 = (status1) & (~previousStatusBoard1);
    if (newTouches1) {
      Serial.println("Touch detected: Board 1 (0x5A)");
    }
    previousStatusBoard1 = status1;
  }

  if (ok2) {
    uint16_t newTouches2 = (status2) & (~previousStatusBoard2);
    if (newTouches2) {
      Serial.println("Touch detected: Board 2 (0x5B)");
    }
    previousStatusBoard2 = status2;
  }

  delay(10);
}

