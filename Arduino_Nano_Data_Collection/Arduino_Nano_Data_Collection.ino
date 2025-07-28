#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// --- IMU setup ---
Adafruit_BNO055 imuFront = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 imuRear  = Adafruit_BNO055(55, 0x29);

sensors_event_t frontData, rearData;
imu::Vector<3> accelFront, accelRear;

// Hall sensor pins
constexpr uint8_t HALL_FRONT_PIN = 2;
constexpr uint8_t HALL_REAR_PIN = 3;
constexpr uint8_t YAW_RESET = 4;

volatile bool sensorError = false;

// Number of magnets on wheel, for RPM calculation
constexpr uint8_t NUM_MAGNETS = 4;
constexpr unsigned long STOP_THRESHOLD_US = 2000000UL; // If 2 seconds are exceeded without a magnet then we assume ZERO RPM
constexpr unsigned long DEBOUNCE_US = 2000UL;

// RPM measurement variables
volatile unsigned long lastPulseTimeFront = 0, pulseIntervalFront = 0, lastDebounceFront = 0;
volatile unsigned long lastPulseTimeRear  = 0, pulseIntervalRear  = 0, lastDebounceRear  = 0;

float rpmFront = 0, rpmRear = 0;

// Yaw biases (to zero‐out each IMU’s heading on button‐press)
float frontYawBias = 0;
float rearYawBias  = 0;

// Keep track of last time we actually zeroed the yaw (ms)
unsigned long lastResetTime = 0;

// We’ll poll pin 4 in loop()—track its previous state here
int lastResetPinState = HIGH;

// --- CALIBRATION OFFSETS ---
adafruit_bno055_offsets_t sensorOffsets = {
  /* accel_offset */  -16,  20,  -14,
  /* mag_offset   */    0,   0,    0, 
  /* gyro_offset  */   -2,  -2,    1,
  /* accel_radius */ 1000,
  /* mag_radius   */  480
};


// Forward declarations
void updateRPM();
void frontPulse();
void rearPulse();
void resetYaw();

void setup() {
  // Yaw-reset “button” (we’ll poll this pin in loop())
  pinMode(YAW_RESET, INPUT_PULLUP);
  lastResetPinState = digitalRead(YAW_RESET);

  Serial.begin(57600);
  delay(1000);

  // Initialize both IMUs
  if (!imuFront.begin() || !imuRear.begin()) {
    sensorError = true;
  }

  // Use the external 32 kHz crystal for better stability
  imuFront.setExtCrystalUse(true);
  imuRear.setExtCrystalUse(true);
  delay(500);

  // Put them into CONFIG mode so we can write offsets
  imuFront.setMode(OPERATION_MODE_CONFIG);
  imuRear.setMode(OPERATION_MODE_CONFIG);
  delay(500);

  // Write pre‐measured calibration offsets into each BNO055
  imuFront.setSensorOffsets(sensorOffsets);
  imuRear.setSensorOffsets(sensorOffsets);
  delay(500);

  // Now go into IMUPLUS mode (gyro + accel only).
  // If you want full 9-DOF fusion, replace with OPERATION_MODE_NDOF.
  imuFront.setMode(OPERATION_MODE_IMUPLUS);
  imuRear.setMode(OPERATION_MODE_IMUPLUS);
  delay(500);

  // Setup Hall sensors for wheel RPM (they still use interrupts)
  pinMode(HALL_FRONT_PIN, INPUT_PULLUP);
  pinMode(HALL_REAR_PIN,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL_FRONT_PIN), frontPulse, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_REAR_PIN),  rearPulse,  CHANGE);
}

void loop() {
  // 1) Manually check for a FALL on pin 4 (YAW_RESET)
  int currentResetState = digitalRead(YAW_RESET);
  if (lastResetPinState == HIGH && currentResetState == LOW) {
    // Detected HIGH→LOW transition → call resetYaw()
    resetYaw();
  }
  lastResetPinState = currentResetState;

  // 2) Handle incoming serial commands from ESP32 (or USB)
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'R') {
      // Update RPM readings
      updateRPM();

      // Read orientation & linear acceleration from both IMUs
      imuFront.getEvent(&frontData);
      imuRear.getEvent(&rearData);

      accelFront = imuFront.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      accelRear  = imuRear .getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      // Subtract each IMU’s bias from its current heading
      float frontYaw = frontData.orientation.x - frontYawBias;
      float rearYaw  = rearData.orientation.x  - rearYawBias;

      // --- Wrap into –180…+180 ---
      if (frontYaw >  180.0f)  frontYaw -= 360.0f;
      if (frontYaw <= -180.0f) frontYaw += 360.0f;

      if (rearYaw >   180.0f)  rearYaw -= 360.0f;
      if (rearYaw <=  -180.0f) rearYaw += 360.0f;

      Serial.print(frontYaw);                       Serial.print(",");
      Serial.print(frontData.orientation.y);        Serial.print(",");
      Serial.print(frontData.orientation.z);        Serial.print(",");
      Serial.print(rearYaw);                        Serial.print(",");
      Serial.print(rearData.orientation.y);         Serial.print(",");
      Serial.print(rearData.orientation.z);         Serial.print(",");
      Serial.print(accelFront.x());                 Serial.print(",");
      Serial.print(accelFront.y());                 Serial.print(",");
      Serial.print(accelFront.z());                 Serial.print(",");
      Serial.print(accelRear.x());                  Serial.print(",");
      Serial.print(accelRear.y());                  Serial.print(",");
      Serial.print(accelRear.z());                  Serial.print(",");
      Serial.print(rpmFront);                       Serial.print(",");
      Serial.println(rpmRear);
    }
    else if (command == 'C') {
      // Report only gyro/accel calibration for front & rear
      uint8_t sysF, gyroF, accelF, magF;
      uint8_t sysR, gyroR, accelR, magR;
      imuFront.getCalibration(&sysF,  &gyroF,  &accelF,  &magF);
      imuRear .getCalibration(&sysR,  &gyroR,  &accelR,  &magR);

      // Print gyroFront, accelFront, gyroRear, accelRear
      Serial.print(gyroF);  Serial.print(",");
      Serial.print(accelF); Serial.print(",");
      Serial.print(gyroR);  Serial.print(",");
      Serial.println(accelR);
    }
    else if (command == 'E') {
      if (sensorError) {
        Serial.println("ERROR");
      }
      else {
        Serial.println("NO ERROR");
      }
    }
  }
}

void updateRPM() {
  unsigned long now = micros();

  // FRONT wheel
  if (now - lastPulseTimeFront > STOP_THRESHOLD_US) {
    rpmFront = 0;
  } else if (pulseIntervalFront > 0) {
    float period = pulseIntervalFront * 1e-6f * NUM_MAGNETS; // seconds per revolution
    rpmFront = 60.0f / period;
  }

  // REAR wheel
  if (now - lastPulseTimeRear > STOP_THRESHOLD_US) {
    rpmRear = 0;
  } else if (pulseIntervalRear > 0) {
    float period = pulseIntervalRear * 1e-6f * NUM_MAGNETS;
    rpmRear = 60.0f / period;
  }
}

void frontPulse() {
  unsigned long now = micros();
  if (now - lastDebounceFront < DEBOUNCE_US) return;
  lastDebounceFront = now;
  pulseIntervalFront = now - lastPulseTimeFront;
  lastPulseTimeFront = now;
}

void rearPulse() {
  unsigned long now = micros();
  if (now - lastDebounceRear < DEBOUNCE_US) return;
  lastDebounceRear = now;
  pulseIntervalRear = now - lastPulseTimeRear;
  lastPulseTimeRear = now;
}

void resetYaw() {
  unsigned long now = millis();
  // Only reset if ≥1000 ms have passed since lastResetTime
  if (now - lastResetTime < 1000UL) {
    return;
  }
  lastResetTime = now;

  // Read current heading from each IMU and store as the new “zero”
  imuFront.getEvent(&frontData);
  imuRear .getEvent(&rearData);

  frontYawBias = frontData.orientation.x;
  rearYawBias  = rearData.orientation.x;
}
