/*
  Enhanced Line Follower (No PID: proportional only)
  Hardware:
    Arduino Uno + Adafruit Motor Shield (L293D v1)
    Motors on M3 (LEFT) and M4 (RIGHT) by default (adjust if needed)
    5 analog IR line sensors on A0..A4 (left -> right). Change order if reversed.
    Optional CLP on D7, NEAR on A5.
*/

#include <AFMotor.h>

// ---------------- USER CONFIG ----------------
const uint8_t NUM_SENSORS = 5;
uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4}; // Ensure index 0 = LEFT sensor

// Motor objects (Adafruit L293D)
AF_DCMotor leftMotor(3);
AF_DCMotor rightMotor(4);

// Motor direction overrides (FORWARD or BACKWARD) to correct wiring
uint8_t LEFT_DIR  = FORWARD;
uint8_t RIGHT_DIR = FORWARD; // Change to BACKWARD if right side spins opposite correct forward direction

// Optional pins
const int CLP_PIN  = 7;      // -1 if unused
const int NEAR_PIN = A5;     // -1 if unused

// Speeds and control
int   BASE_SPEED           = 40;  // Nominal straight speed
int   MAX_SPEED            = 85;
float KP                   = 0.07; // Proportional gain (adjust)
bool  REVERSE_ERROR        = false; // Set true if it turns opposite

// Adaptive speed: reduce speed when error large
int   MIN_SPEED_AT_LARGE_ERR = 80;
int   LARGE_ERROR_THRESHOLD  = 1200; // (position units ~0..4000, center=2000)
bool  USE_ADAPTIVE_SPEED     = true;

// Line polarity (if black gives lower raw => BLACK_LOW = true)
bool  BLACK_LOW = true;          // Will be overwritten if AUTO_POLARITY enabled
bool  ENABLE_AUTO_POLARITY = false; // Prompts user at start (needs you to place sensor over white then black)

// Speed ramp at startup
bool  USE_STARTUP_RAMP = true;
unsigned long RAMP_TIME_MS = 3000; // time to reach BASE_SPEED

// Lost line behavior
int LOST_LINE_TURN_SPEED      = 100;
unsigned long LOST_LINE_TIMEOUT_MS = 400;

// Obstacle logic (NEAR sensor)
bool USE_NEAR         = false;
int  NEAR_THRESHOLD   = 500;
int  NEAR_SLOW_SPEED  = 90;

// Debug
unsigned long DEBUG_INTERVAL_MS = 300;
bool SHOW_NORMALIZED = true;

// Calibration time
unsigned long CALIBRATION_TIME_MS = 3000;

// ---------------- INTERNAL STATE ----------------
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

unsigned long lastLineMillis = 0;
int lastError = 0;
unsigned long startMillis;

unsigned long lastDebug = 0;

// ---------------- UTILITIES ----------------
void motorSet(int left, int right) {
  left  = constrain(left,  0, 55);
  right = constrain(right, 0, 55);
  leftMotor.setSpeed(left);
  rightMotor.setSpeed(right);
  leftMotor.run(LEFT_DIR);
  rightMotor.run(RIGHT_DIR);
}

void motorStop() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

void pulseCLP() {
  if (CLP_PIN >= 0) {
    digitalWrite(CLP_PIN, HIGH);
    delay(3);
    digitalWrite(CLP_PIN, LOW);
  }
}

void readRaw(int rawVals[]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    rawVals[i] = analogRead(sensorPins[i]);
  }
}

void readNormalized(int normVals[]) {
  int raw[NUM_SENSORS];
  readRaw(raw);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int span = sensorMax[i] - sensorMin[i];
    if (span < 5) span = 5;
    long v = (long)(raw[i] - sensorMin[i]) * 1000L / span;
    if (v < 0) v = 0;
    if (v > 1000) v = 1000;
    if (BLACK_LOW) {
      // raw black ~ sensorMin => low v => invert so black is HIGH for weighting
      v = 1000 - v;
    }
    normVals[i] = (int)v;
  }
}

long computePosition(int normVals[], bool &found) {
  long numerator = 0;
  long denom = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int w = normVals[i];
    if (w > 50) { // threshold
      numerator += (long)w * (i * 1000L);
      denom += w;
    }
  }
  if (denom == 0) {
    found = false;
    return -1;
  }
  found = true;
  return numerator / denom;
}

int effectiveBaseSpeed(int error) {
  if (!USE_ADAPTIVE_SPEED) return BASE_SPEED;
  int aerr = abs(error);
  if (aerr >= LARGE_ERROR_THRESHOLD) return MIN_SPEED_AT_LARGE_ERR;
  // Linear interpolate between BASE_SPEED and MIN_SPEED_AT_LARGE_ERR
  float f = (float)aerr / LARGE_ERROR_THRESHOLD;
  int sp = BASE_SPEED - (int)((BASE_SPEED - MIN_SPEED_AT_LARGE_ERR) * f);
  return sp;
}

void applyProportional(int error) {
  if (REVERSE_ERROR) error = -error;
  // Adaptive base speed
  int base = effectiveBaseSpeed(error);
  float corr = KP * error;
  int left = base - (int)corr;
  int right = base + (int)corr;
  left = constrain(left, 0, MAX_SPEED);
  right = constrain(right, 0, MAX_SPEED);
  motorSet(left, right);
  lastError = error;
}

void lostLineBehavior() {
  unsigned long dt = millis() - lastLineMillis;
  if (dt > LOST_LINE_TIMEOUT_MS) {
    // Search: turn toward last known direction
    if (lastError >= 0) {
      motorSet(LOST_LINE_TURN_SPEED, 0);
    } else {
      motorSet(0, LOST_LINE_TURN_SPEED);
    }
  } else {
    // Short drift straight
    motorSet(80, 80);
  }
}

void calibrate() {
  Serial.println(F("Calibrating... Move robot so each sensor sees BOTH black line and white area."));
  unsigned long t0 = millis();
  while (millis() - t0 < CALIBRATION_TIME_MS) {
    int raw[NUM_SENSORS];
    readRaw(raw);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
    if (CLP_PIN >= 0) pulseCLP();
    delay(5);
  }
  Serial.println(F("Calibration complete (min-max per sensor):"));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i); Serial.print(": ");
    Serial.print(sensorMin[i]); Serial.print(" - ");
    Serial.println(sensorMax[i]);
  }
}

void autoPolarityPrompt() {
  Serial.println(F("AUTO POLARITY: Place CENTER sensor over WHITE area. Press reset if wrong."));
  delay(1500);
  int whiteVal = analogRead(sensorPins[NUM_SENSORS / 2]);
  Serial.print(F("White sample = ")); Serial.println(whiteVal);
  Serial.println(F("Now place CENTER sensor over BLACK LINE."));
  delay(2000);
  int blackVal = analogRead(sensorPins[NUM_SENSORS / 2]);
  Serial.print(F("Black sample = ")); Serial.println(blackVal);
  if (blackVal < whiteVal) {
    BLACK_LOW = true;
    Serial.println(F("Detected: BLACK gives LOWER reading. Setting BLACK_LOW = true."));
  } else {
    BLACK_LOW = false;
    Serial.println(F("Detected: BLACK gives HIGHER reading. Setting BLACK_LOW = false."));
  }
  delay(1000);
}

int rampedSpeed() {
  if (!USE_STARTUP_RAMP) return BASE_SPEED;
  unsigned long elapsed = millis() - startMillis;
  if (elapsed >= RAMP_TIME_MS) return BASE_SPEED;
  float f = (float)elapsed / RAMP_TIME_MS;
  int startSpeed = 60; // initial small motion
  return startSpeed + (int)((BASE_SPEED - startSpeed) * f);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n--- Line Follower Start ---"));

  if (CLP_PIN >= 0) {
    pinMode(CLP_PIN, OUTPUT);
    digitalWrite(CLP_PIN, LOW);
  }
  if (NEAR_PIN >= 0) {
    pinMode(NEAR_PIN, INPUT);
  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  if (ENABLE_AUTO_POLARITY) {
    autoPolarityPrompt();
  } else {
    Serial.print(F("BLACK_LOW (manual) = "));
    Serial.println(BLACK_LOW ? F("true") : F("false"));
  }

  calibrate();
  startMillis = millis();
  lastLineMillis = millis();
  Serial.println(F("Entering run loop..."));
}

// ---------------- LOOP ----------------
void loop() {
  // Adaptive ramp of base speed
  int dynamicBase = rampedSpeed();

  // Override BASE_SPEED only internally; the configured BASE_SPEED is still used beyond ramp
  int savedBase = BASE_SPEED;
  BASE_SPEED = dynamicBase;

  // Obstacle slow down
  if (USE_NEAR && NEAR_PIN >= 0) {
    int nearVal = analogRead(NEAR_PIN);
    if (nearVal > NEAR_THRESHOLD) {
      BASE_SPEED = NEAR_SLOW_SPEED;
    }
  }

  int norm[NUM_SENSORS];
  readNormalized(norm);
  bool found;
  long pos = computePosition(norm, found);

  if (!found) {
    lostLineBehavior();
  } else {
    lastLineMillis = millis();
    long center = (long)(NUM_SENSORS - 1) * 1000L / 2L; // For 5 sensors -> 2000
    int error = (int)(pos - center);
    applyProportional(error);
  }

  // Debug output
  if (millis() - lastDebug >= DEBUG_INTERVAL_MS) {
    lastDebug = millis();
    Serial.print(F("Found=")); Serial.print(found);
    Serial.print(F(" Pos=")); Serial.print(pos);
    Serial.print(F(" Err=")); Serial.print(lastError);
    Serial.print(F(" BaseEff=")); Serial.print(BASE_SPEED);
    if (SHOW_NORMALIZED) {
      Serial.print(F(" Norm:["));
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(norm[i]);
        if (i < NUM_SENSORS - 1) Serial.print(' ');
      }
      Serial.print(']');
    }
    Serial.println();
  }

  // Restore original base (for next loop ramp calculation)
  BASE_SPEED = savedBase;
}