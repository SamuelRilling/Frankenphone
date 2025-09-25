/*
  Capacitive Touch Instrument with Dual MPR121 on ESP32

  Hardware:
  - ESP32
  - MPR121 #1 at I2C address 0x5A on SDA=GPIO13, SCL=GPIO14 (separate I2C bus)
  - MPR121 #2 at I2C address 0x5B on SDA=GPIO27, SCL=GPIO26 (separate I2C bus)
  - Audio output on GPIO25 using LEDC PWM (ledcWriteTone)

  Requirements addressed:
  - Safe initialization of both MPR121s; if one is missing, continue without crashes or resets
  - No infinite boot loops; sensors may be missing at boot and later attached
  - Detect pad touch events from both boards
  - Print debug only when state changes (no continuous loop spam)
  - Map touches to specific frequencies and play tones
  - Use non-blocking I2C with timeouts; avoid invalid memory access when sensors absent
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPR121.h>

// -----------------------------
// Pin and configuration defines
// -----------------------------
static const uint8_t I2C0_SDA_PIN = 13;   // Bus for MPR121 at 0x5A
static const uint8_t I2C0_SCL_PIN = 14;
static const uint8_t I2C1_SDA_PIN = 27;   // Bus for MPR121 at 0x5B
static const uint8_t I2C1_SCL_PIN = 26;

static const uint8_t MPR121_ADDR_A = 0x5A;  // Default
static const uint8_t MPR121_ADDR_B = 0x5B;  // ADDR tied to 3.3V

static const uint32_t I2C_FREQUENCY_HZ = 400000; // Fast-mode I2C
static const uint32_t I2C_TIMEOUT_MS = 50;       // Avoid long blocking

// Audio (LEDC PWM) on ESP32 (Arduino-ESP32 v3 pin-based API)
static const uint8_t TONE_PIN = 25;
static const uint8_t LEDC_RES_BITS = 10;   // 10-bit resolution
static const uint32_t LEDC_BASE_FREQ = 1000; // Initial attach frequency

static const uint8_t NUM_PADS = 12;        // MPR121 has 12 electrodes

// -----------------------------
// Global objects
// -----------------------------
TwoWire I2C_BUS_A = TwoWire(0);  // I2C bus for 0x5A on pins 13/14
TwoWire I2C_BUS_B = TwoWire(1);  // I2C bus for 0x5B on pins 27/26

Adafruit_MPR121 mprA;
Adafruit_MPR121 mprB;

bool sensorA_ok = false;
bool sensorB_ok = false;

uint16_t lastTouchedA = 0;
uint16_t lastTouchedB = 0;
uint16_t currTouchedA = 0;
uint16_t currTouchedB = 0;

int currentFrequency = 0;  // Track currently playing frequency to avoid redundant reconfig

// Frequencies mapped per pad (C-major scale across two octaves as example)
// Board A (0x5A): approx C4..G5
static const int PAD_FREQUENCIES_A[NUM_PADS] = {
  262, 294, 330, 349, 392, 440, 494,  // C4, D4, E4, F4, G4, A4, B4
  523, 587, 659, 698, 784             // C5, D5, E5, F5, G5
};

// Board B (0x5B): approx C5..G6
static const int PAD_FREQUENCIES_B[NUM_PADS] = {
  523, 587, 659, 698, 784, 880, 988,  // C5, D5, E5, F5, G5, A5, B5
  1047, 1175, 1319, 1397, 1568        // C6, D6, E6, F6, G6
};

// -----------------------------
// Helpers: audio control
// -----------------------------
void stopTone()
{
  // Silence by setting duty to 0
  ledcWrite(TONE_PIN, 0);
  currentFrequency = 0;
}

void startTone(int frequencyHz)
{
  if (frequencyHz <= 0) {
    stopTone();
    return;
  }

  if (currentFrequency == frequencyHz) {
    return; // Already playing this frequency
  }

  // Configure tone frequency; duty will remain whatever last set by ledcWrite
  ledcWriteTone(TONE_PIN, (double)frequencyHz);

  // Set duty to 50% of max for audible volume (scales with resolution)
  const uint32_t maxDuty = (1UL << LEDC_RES_BITS) - 1;
  ledcWrite(TONE_PIN, maxDuty / 2);

  currentFrequency = frequencyHz;
}

int selectActiveFrequency()
{
  // Priority: lowest pad index on board A, then lowest on board B
  for (uint8_t i = 0; i < NUM_PADS; i++) {
    if (currTouchedA & (1 << i)) {
      return PAD_FREQUENCIES_A[i];
    }
  }
  for (uint8_t i = 0; i < NUM_PADS; i++) {
    if (currTouchedB & (1 << i)) {
      return PAD_FREQUENCIES_B[i];
    }
  }
  return 0; // none active
}

// -----------------------------
// Helpers: sensor processing
// -----------------------------
bool processSensor(const char* label,
                   bool sensorOk,
                   Adafruit_MPR121& sensor,
                   uint16_t& lastState,
                   uint16_t& currState)
{
  if (!sensorOk) {
    currState = 0;
    return false;
  }

  uint16_t touched = sensor.touched();
  currState = touched;

  uint16_t changed = touched ^ lastState;
  if (changed) {
    for (uint8_t i = 0; i < NUM_PADS; i++) {
      uint16_t mask = (1 << i);
      if (changed & mask) {
        if (touched & mask) {
          Serial.print("[MPR121 "); Serial.print(label); Serial.print("] Pad "); Serial.print(i); Serial.println(" TOUCH");
        } else {
          Serial.print("[MPR121 "); Serial.print(label); Serial.print("] Pad "); Serial.print(i); Serial.println(" RELEASE");
        }
      }
    }
    lastState = touched; // update last after reporting
    return true;
  }

  return false;
}

// -----------------------------
// Setup and loop
// -----------------------------
void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Capacitive Touch Instrument (ESP32 + dual MPR121)");

  // Initialize I2C buses (separate for each sensor)
  I2C_BUS_A.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, I2C_FREQUENCY_HZ);
  I2C_BUS_A.setTimeOut(I2C_TIMEOUT_MS);

  I2C_BUS_B.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, I2C_FREQUENCY_HZ);
  I2C_BUS_B.setTimeOut(I2C_TIMEOUT_MS);

  // Initialize LEDC for audio (v3 API)
  if (!ledcAttach(TONE_PIN, LEDC_BASE_FREQ, LEDC_RES_BITS)) {
    Serial.println("LEDC attach failed on pin 25");
  }
  stopTone();

  // Initialize sensors. If a sensor is missing, continue without resetting.
  sensorA_ok = mprA.begin(MPR121_ADDR_A, &I2C_BUS_A);
  if (sensorA_ok) {
    // Set reasonable thresholds (touch, release)
    mprA.setThresholds(12, 6);
    Serial.println("MPR121 A (0x5A) detected and configured.");
  } else {
    Serial.println("MPR121 A (0x5A) NOT detected. Continuing without it.");
  }

  sensorB_ok = mprB.begin(MPR121_ADDR_B, &I2C_BUS_B);
  if (sensorB_ok) {
    mprB.setThresholds(12, 6);
    Serial.println("MPR121 B (0x5B) detected and configured.");
  } else {
    Serial.println("MPR121 B (0x5B) NOT detected. Continuing without it.");
  }
}

void loop()
{
  // Process each sensor and emit logs only on changes
  bool changedA = processSensor("A", sensorA_ok, mprA, lastTouchedA, currTouchedA);
  bool changedB = processSensor("B", sensorB_ok, mprB, lastTouchedB, currTouchedB);

  if (changedA || changedB) {
    // Update tone selection based on current set of active touches
    int freq = selectActiveFrequency();
    if (freq > 0) {
      startTone(freq);
    } else {
      stopTone();
    }
  }

  // Small delay to reduce I2C bus pressure and CPU use
  delay(8);
}

