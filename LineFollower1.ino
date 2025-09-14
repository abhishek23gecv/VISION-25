/*
  LineFollower.ino
  Arduino Uno + Adafruit Motor Shield (L293D v1) + 5-analog IR sensor array.
  Motors on M3 (left) and M4 (right).
  Sensors S1..S5 -> A0..A4.
  Optional: CLP -> D7 (if your board supports a clamp/calibrate pulse).
  Optional: NEAR -> A5 (obstacle / proximity).
  No PID (only proportional correction).
*/

#include <AFMotor.h>

// ------------- Configuration -------------
const uint8_t NUM_SENSORS = 5;
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4};

const int CLP_PIN  = 7;     // Set to -1 if not used
const int NEAR_PIN = A5;    // Set to -1 if not used

// Timing
const unsigned long CALIBRATION_TIME_MS = 2000;

// Movement parameters
int BASE_SPEED = 50;        // Base motor speed (0-255)
int MAX_SPEED  = 85;
float KP       = 0.08f;      // Proportional gain (adjust for responsiveness)
int LOST_LINE_TURN_SPEED = 110;
unsigned long LOST_LINE_TIMEOUT_MS = 350;

// Optional obstacle slow-down
bool USE_NEAR = true;
int  NEAR_THRESHOLD = 500;   // Adjust after printing near sensor raw
int  NEAR_SLOW_SPEED = 100;  // Reduced base speed near obstacle

// Inversion: If black line gives LOWER analog reading (typical reflective),
// set BLACK_LOW = true. If opposite, set false.
bool BLACK_LOW = true;

// ------------- Internal State -------------
AF_DCMotor leftMotor(3);
AF_DCMotor rightMotor(4);

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

unsigned long lastLineDetectedMillis = 0;
int lastError = 0;

// ------------- Utility Functions -------------
void setMotorSpeeds(int left, int right) {
  left = constrain(left, 0, 50);
  right = constrain(right, 0, 50);
  leftMotor.setSpeed(left);
  rightMotor.setSpeed(right);
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
}

void applyCorrection(int error) {
  // Proportional-only (not full PID)
  float correction = KP * error;
  int left = BASE_SPEED - (int)correction;
  int right = BASE_SPEED + (int)correction;
  left = constrain(left, 0, MAX_SPEED);
  right = constrain(right, 0, MAX_SPEED);
  setMotorSpeeds(left, right);
}

// Reads raw analog values
void readRaw(int rawVals[]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    rawVals[i] = analogRead(sensorPins[i]);
  }
}

// Normalize to 0..1000 after calibration
void readNormalized(int normVals[]) {
  int raw[NUM_SENSORS];
  readRaw(raw);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int span = sensorMax[i] - sensorMin[i];
    if (span < 5) span = 5; // Prevent div by near-zero
    long val = (long)(raw[i] - sensorMin[i]) * 1000L / span;
    if (val < 0) val = 0;
    if (val > 1000) val = 1000;
    if (BLACK_LOW) {
      // If black is low reflectance, raw on black is near sensorMin.
      // After normalizing above: black ~0, white ~1000.
      // For line position we want bigger number where the line is:
      // So invert
      val = 1000 - val;
    }
    normVals[i] = (int)val;
  }
}

// Returns: weighted position 0..(NUM_SENSORS-1)*1000, or -1 if no line
long getLinePosition(int normVals[], bool &lineFound) {
  long numerator = 0;
  long denominator = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int w = normVals[i]; // already bigger where line likely
    if (w > 50) { // threshold to count
      numerator += (long)w * (i * 1000L);
      denominator += w;
    }
  }
  if (denominator == 0) {
    lineFound = false;
    return -1;
  }
  lineFound = true;
  return numerator / denominator;
}

// Lost line handling: gentle search turn
void lostLineBehavior() {
  // Simple approach: turn slightly in direction of last error
  if (millis() - lastLineDetectedMillis > LOST_LINE_TIMEOUT_MS) {
    // Harder search if gone longer
    if (lastError >= 0) {
      setMotorSpeeds(LOST_LINE_TURN_SPEED, 0);
    } else {
      setMotorSpeeds(0, LOST_LINE_TURN_SPEED);
    }
  } else {
    // Briefly keep last correction (coast straight)
    setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  }
}

// Optional discrete fallback (not used by default)
void discreteLogic(int normVals[]) {
  // Example simple logic if you ever want pure if/else:
  bool s0 = normVals[0] > 400;
  bool s1 = normVals[1] > 400;
  bool s2 = normVals[2] > 400;
  bool s3 = normVals[3] > 400;
  bool s4 = normVals[4] > 400;

  if (s2 && !s1 && !s3) {
    setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  } else if (s1 || (s0 && !s4)) {
    setMotorSpeeds(BASE_SPEED / 2, BASE_SPEED);
  } else if (s3 || (s4 && !s0)) {
    setMotorSpeeds(BASE_SPEED, BASE_SPEED / 2);
  } else if (s0) {
    setMotorSpeeds(BASE_SPEED / 4, BASE_SPEED);
  } else if (s4) {
    setMotorSpeeds(BASE_SPEED, BASE_SPEED / 4);
  } else {
    lostLineBehavior();
  }
}

void pulseCLP() {
  if (CLP_PIN >= 0) {
    digitalWrite(CLP_PIN, HIGH);
    delay(5);
    digitalWrite(CLP_PIN, LOW);
  }
}

void calibrateSensors() {
  Serial.println(F("Calibrating... Move robot so each sensor sees both line and background."));
  unsigned long start = millis();
  while (millis() - start < CALIBRATION_TIME_MS) {
    int raw[NUM_SENSORS];
    readRaw(raw);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
    if (CLP_PIN >= 0) {
      // If needed to periodically clamp (depends on board)
      pulseCLP();
    }
    delay(5);
  }
  Serial.println(F("Calibration done. Min/Max:"));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i); Serial.print(": ");
    Serial.print(sensorMin[i]); Serial.print(" - ");
    Serial.println(sensorMax[i]);
  }
}

// ------------- Setup & Loop -------------
void setup() {
  Serial.begin(115200);

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);

  if (CLP_PIN >= 0) {
    pinMode(CLP_PIN, OUTPUT);
    digitalWrite(CLP_PIN, LOW);
  }
  if (NEAR_PIN >= 0) {
    pinMode(NEAR_PIN, INPUT);
  }

  // Init calibration arrays
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  delay(500);
  calibrateSensors();
  lastLineDetectedMillis = millis();
  Serial.println(F("Starting line following..."));
}

void loop() {
  // Optional dynamic near obstacle slow-down
  if (USE_NEAR && NEAR_PIN >= 0) {
    int nearVal = analogRead(NEAR_PIN);
    // Print occasionally
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      Serial.print(F("NEAR raw: "));
      Serial.println(nearVal);
      lastPrint = millis();
    }
    if (nearVal > NEAR_THRESHOLD) {
      BASE_SPEED = NEAR_SLOW_SPEED;
    } else {
      BASE_SPEED = 140; // restore default
    }
  }

  int normVals[NUM_SENSORS];
  readNormalized(normVals);

  bool lineFound;
  long position = getLinePosition(normVals, lineFound);

  if (!lineFound) {
    lostLineBehavior();
    return;
  }

  lastLineDetectedMillis = millis();

  // Center is (NUM_SENSORS - 1) * 1000 / 2
  long center = (long)(NUM_SENSORS - 1) * 1000L / 2L; // For 5 sensors -> 2000
  int error = (int)(position - center);

  // Weighted proportional correction (not full PID)
  applyCorrection(error);
  lastError = error;

  // Debug output (comment out for performance)
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 300) {
    Serial.print(F("Pos: ")); Serial.print(position);
    Serial.print(F(" Err: ")); Serial.print(error);
    Serial.print(F(" Norm: "));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      Serial.print(normVals[i]); Serial.print(' ');
    }
    Serial.println();
    lastDbg = millis();
  }

  // If you want to test discrete logic instead of weighted method:
  // discreteLogic(normVals);
}