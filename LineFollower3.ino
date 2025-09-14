/*
  High-Precision PID Line Follower
  Hardware:
    - Arduino Uno
    - Adafruit Motor Shield L293D (v1) using AFMotor library
    - 5 analog IR line sensors (left->right) on A0..A4
    - Optional CLP pin for sensor array (exposure latch) on D7
    - Optional NEAR (obstacle proximity) on A5
  Motors attached to M3 (LEFT) and M4 (RIGHT) by default.

  Features:
    - Calibration of min/max per sensor
    - Automatic or manual polarity (BLACK_LOW)
    - Moving average smoothing
    - Weighted position 0..(N-1)*1000 (for 5 sensors -> 0..4000)
    - Full PID with anti-windup and derivative filtering
    - Adaptive base speed vs error magnitude
    - Lost line detection + recovery spin
    - Optional obstacle slow-down
    - Junction detection helper (commented)
    - Extensive debug toggles

  NOTE:
    - Real "100% accuracy" is not feasible; environment matters.
    - Tuning required. Start slowly, increase speed later.

  Author: (Generated for your scenario)
*/

#include <AFMotor.h>

// ========================== USER CONFIG ==========================

// --- Sensor & Pins ---
const uint8_t NUM_SENSORS = 5;
uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4}; // index 0 MUST be left-most physically
const int CLP_PIN  = 7;   // -1 if unused
const int NEAR_PIN = A5;  // -1 if unused

// --- Motor Setup ---
AF_DCMotor leftMotor(3);
AF_DCMotor rightMotor(4);

// If a motor runs backwards when you command forward, flip its direction constant:
uint8_t LEFT_DIR_DEFAULT  = FORWARD;
uint8_t RIGHT_DIR_DEFAULT = FORWARD;
// If you need to swap: e.g., RIGHT_DIR_DEFAULT = BACKWARD;

// --- Polarity ---
bool ENABLE_AUTO_POLARITY = false; // true => prompts user
bool BLACK_LOW = true;             // if black reflectance -> lower raw value (common)
bool REVERSE_ERROR = false;        // if turning direction is inverted, set true

// --- PID Gains ---
float Kp = 0.13f;
float Ki = 0.00055f;
float Kd = 2.2f;

// Derivative filtering factor: 0 = no filtering, 0.7 moderate, 0.9 heavy smoothing
float DERIVATIVE_FILTER = 0.6f;

// If true derivative uses change in error; if false uses change in position (measurement) style
bool DERIV_ON_ERROR = true;

// Integral limits & conditions
float I_TERM = 0.0f;
float I_MAX = 300.0f;       // clamp absolute integral contribution
bool INTEGRAL_WHEN_LOST = false; // do NOT integrate when line lost (usually false)

// --- Speeds ---
int BASE_SPEED = 40;       // nominal forward speed
int MAX_SPEED  = 65;
int MIN_BASE_SPEED = 90;    // base speed minimum used for adaptive scaling
int STARTUP_RAMP_TIME_MS = 2000;
bool USE_STARTUP_RAMP = true;

// Adaptive speed vs error magnitude
bool ENABLE_ADAPTIVE_BASE = true;
int ERROR_FOR_MIN_BASE = 1400;  // |error| above which base speed hits MIN_BASE_SPEED

// --- Smoothing / Sampling ---
const uint8_t SMOOTH_SAMPLES = 4;  // moving average window (>=2)
int rawHistory[NUM_SENSORS][SMOOTH_SAMPLES];
uint8_t historyIndex = 0;

// --- Calibration ---
unsigned long CALIBRATION_TIME_MS = 3000;

// --- Lost Line Handling ---
unsigned long lastLineMillis = 0;
unsigned long LOST_LINE_GRACE_MS = 250;  // after losing line keep going straight
unsigned long LOST_LINE_SEARCH_MS = 800; // after grace, start searching
int LOST_LINE_SEARCH_SPEED = 110;
bool lastLineDetected = true;
int lastError = 0;

// --- Obstacle (NEAR) ---
bool USE_NEAR = false;
int NEAR_THRESHOLD = 600;
int NEAR_REDUCED_SPEED = 100;

// --- Junction Detection (optional) ---
bool ENABLE_JUNCTION_HINT = false;
int JUNCTION_THRESHOLD = 850; // if multiple sensors all high (line broad/cross)
// (Add your own logic after reading normalized values.)

// --- Debugging ---
bool DEBUG_ENABLED = true;
unsigned long DEBUG_INTERVAL_MS = 300;
bool SHOW_NORMALIZED = true;
bool SHOW_RAW = false;
bool SHOW_PID_TERMS = true;

// ========================== INTERNAL STATE =======================

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

uint8_t LEFT_DIR  = LEFT_DIR_DEFAULT;
uint8_t RIGHT_DIR = RIGHT_DIR_DEFAULT;

unsigned long startMillis;
unsigned long lastDebugMillis = 0;

float lastFilteredDerivative = 0.0f;
int lastPosition = -1;

// ========================== UTILITY FUNCTIONS ====================

void motorSet(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
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

// Acquire raw analog for each sensor
void readRaw(int rawVals[]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    rawVals[i] = analogRead(sensorPins[i]);
  }
}

// Add current raw reading into moving average history
void updateHistory(const int rawVals[]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    rawHistory[i][historyIndex] = rawVals[i];
  }
  historyIndex = (historyIndex + 1) % SMOOTH_SAMPLES;
}

// Compute averaged (smoothed) raw values
void getSmoothedRaw(int out[]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    long sum = 0;
    for (uint8_t k = 0; k < SMOOTH_SAMPLES; k++)
      sum += rawHistory[i][k];
    out[i] = sum / SMOOTH_SAMPLES;
  }
}

// Convert smoothed raw to normalized 0..1000 (line wants to produce higher weight)
void readNormalized(int normVals[]) {
  int smoothed[NUM_SENSORS];
  getSmoothedRaw(smoothed);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int span = sensorMax[i] - sensorMin[i];
    if (span < 5) span = 5;
    long v = (long)(smoothed[i] - sensorMin[i]) * 1000L / span;
    if (v < 0) v = 0;
    if (v > 1000) v = 1000;

    if (BLACK_LOW) {
      // On black the raw is near sensorMin (low reflect) => after normalization black ~0
      // We invert so black line appears large (closer to 1000) for weighting
      v = 1000 - v;
    }
    normVals[i] = (int)v;
  }
}

// Weighted position 0..(NUM_SENSORS-1)*1000
long computePosition(const int normVals[], bool &found) {
  long numerator = 0;
  long denom = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int weight = normVals[i];
    if (weight > 40) { // threshold
      numerator += (long)weight * (i * 1000L);
      denom += weight;
    }
  }
  if (denom == 0) {
    found = false;
    return -1;
  }
  found = true;
  return numerator / denom;
}

// Adaptive base speed based on |error|
int computeAdaptiveBase(int error) {
  if (!ENABLE_ADAPTIVE_BASE) return BASE_SPEED;
  int aerr = abs(error);
  if (aerr >= ERROR_FOR_MIN_BASE) return MIN_BASE_SPEED;
  // linear ramp
  float f = (float)aerr / ERROR_FOR_MIN_BASE;
  float v = (float)BASE_SPEED - ((float)(BASE_SPEED - MIN_BASE_SPEED) * f);
  return (int)v;
}

// Ramping for startup
int applyStartupRamp(int base) {
  if (!USE_STARTUP_RAMP) return base;
  unsigned long elapsed = millis() - startMillis;
  if (elapsed >= (unsigned long)STARTUP_RAMP_TIME_MS) return base;
  float ratio = (float)elapsed / (float)STARTUP_RAMP_TIME_MS;
  int minStart = 70; // initial crawl
  int ramped = minStart + (int)((base - minStart) * ratio);
  return ramped;
}

// Lost line behavior
void lostLineHandler() {
  unsigned long noLineTime = millis() - lastLineMillis;
  if (noLineTime < LOST_LINE_GRACE_MS) {
    motorSet(80, 80); // coast forward a bit
  } else if (noLineTime < LOST_LINE_GRACE_MS + LOST_LINE_SEARCH_MS) {
    // search spin based on last error sign
    if (lastError >= 0) {
      motorSet(LOST_LINE_SEARCH_SPEED, 0);
    } else {
      motorSet(0, LOST_LINE_SEARCH_SPEED);
    }
  } else {
    // full stop or slow alternate spin
    motorSet(0, 0);
  }
}

// Optional auto polarity
void autoPolaritySequence() {
  Serial.println(F("AUTO POLARITY: Place center sensor over WHITE and keep still..."));
  delay(1500);
  int whiteVal = analogRead(sensorPins[NUM_SENSORS / 2]);
  Serial.print(F("White sample: ")); Serial.println(whiteVal);
  Serial.println(F("Now place center sensor over BLACK line..."));
  delay(2000);
  int blackVal = analogRead(sensorPins[NUM_SENSORS / 2]);
  Serial.print(F("Black sample: ")); Serial.println(blackVal);

  if (blackVal < whiteVal) {
    BLACK_LOW = true;
    Serial.println(F("Detected polarity: black is LOWER -> BLACK_LOW = true"));
  } else {
    BLACK_LOW = false;
    Serial.println(F("Detected polarity: black is HIGHER -> BLACK_LOW = false"));
  }
  delay(500);
}

// Calibration
void calibrateSensors() {
  Serial.println(F("Calibrating... Move robot so each sensor sees BLACK and WHITE."));
  unsigned long t0 = millis();
  while (millis() - t0 < CALIBRATION_TIME_MS) {
    int raw[NUM_SENSORS];
    readRaw(raw);
    updateHistory(raw); // maintain smoothing arrays (even if not stable yet)
    // For min/max we can just use current raw (not yet smoothed)
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
    if (CLP_PIN >= 0) pulseCLP();
    delay(5);
  }
  Serial.println(F("Calibration results (min-max):"));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i); Serial.print(": ");
    Serial.print(sensorMin[i]); Serial.print(" - ");
    Serial.println(sensorMax[i]);
  }
}

// Print debug
void debugPrint(bool lineFound, long position, int error, int baseEff,
                float pTerm, float iTerm, float dTerm, const int normVals[], const int rawVals[]) {
  if (!DEBUG_ENABLED) return;
  unsigned long now = millis();
  if (now - lastDebugMillis < DEBUG_INTERVAL_MS) return;
  lastDebugMillis = now;

  Serial.print(F("LF=")); Serial.print(lineFound);
  Serial.print(F(" Pos=")); Serial.print(position);
  Serial.print(F(" Err=")); Serial.print(error);
  Serial.print(F(" Base=")); Serial.print(baseEff);
  if (SHOW_PID_TERMS) {
    Serial.print(F(" P=")); Serial.print(pTerm, 3);
    Serial.print(F(" I=")); Serial.print(iTerm, 3);
    Serial.print(F(" D=")); Serial.print(dTerm, 3);
  }
  if (SHOW_NORMALIZED) {
    Serial.print(F(" Norm["));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      Serial.print(normVals[i]);
      if (i < NUM_SENSORS - 1) Serial.print(' ');
    }
    Serial.print(']');
  }
  if (SHOW_RAW) {
    Serial.print(F(" Raw["));
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      Serial.print(rawVals[i]);
      if (i < NUM_SENSORS - 1) Serial.print(' ');
    }
    Serial.print(']');
  }
  if (ENABLE_JUNCTION_HINT) {
    // Example heuristic: If 3 or more sensors strongly "see" line => potential junction
    int countHigh = 0;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
      if (normVals[i] > JUNCTION_THRESHOLD) countHigh++;
    Serial.print(F(" Junc?="));
    Serial.print(countHigh >= 3 ? F("Y") : F("N"));
  }
  Serial.println();
}

// ========================== SETUP ================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n--- PID Line Follower Start ---"));

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
    for (uint8_t k = 0; k < SMOOTH_SAMPLES; k++)
      rawHistory[i][k] = 0;
  }

  if (ENABLE_AUTO_POLARITY) {
    autoPolaritySequence();
  } else {
    Serial.print(F("Manual BLACK_LOW = "));
    Serial.println(BLACK_LOW ? F("true") : F("false"));
  }

  calibrateSensors();
  startMillis = millis();
  lastLineMillis = millis();

  Serial.println(F("Entering main loop... Tune Kp,Kd first, then add Ki."));
}

// ========================== LOOP =================================
void loop() {
  // Acquire raw and update smoothing
  int raw[NUM_SENSORS];
  readRaw(raw);
  updateHistory(raw);

  // Compute normalized values
  int norm[NUM_SENSORS];
  readNormalized(norm);

  // Determine line position
  bool lineFound;
  long position = computePosition(norm, lineFound);

  if (!lineFound) {
    lastLineDetected = false;
    // Optionally freeze integral
    if (!INTEGRAL_WHEN_LOST) {
      // Do nothing to I_TERM (or decay)
    }
    lostLineHandler();
    debugPrint(false, position, lastError, BASE_SPEED, 0, I_TERM, 0, norm, raw);
    return;
  }

  if (!lastLineDetected && lineFound) {
    // Just re-acquired line: optional integral reset or partial decay
    I_TERM *= 0.5f; // soften accumulated integral
  }
  lastLineDetected = true;
  lastLineMillis = millis();

  // Center index
  long center = (long)(NUM_SENSORS - 1) * 1000L / 2L; // (5 sensors -> 2000)
  int error = (int)(position - center);
  if (REVERSE_ERROR) error = -error;

  // Adaptive base speed
  int baseEff = computeAdaptiveBase(error);
  baseEff = applyStartupRamp(baseEff);

  // Obstacle slow-down
  if (USE_NEAR && NEAR_PIN >= 0) {
    int nearVal = analogRead(NEAR_PIN);
    if (nearVal > NEAR_THRESHOLD) {
      baseEff = min(baseEff, NEAR_REDUCED_SPEED);
    }
  }

  // PID Terms:
  // P
  float pTerm = Kp * (float)error;

  // I (accumulate only if line found and within sane error range to avoid runaway)
  if (lineFound && (abs(error) < 2500)) {
    I_TERM += Ki * error;
    // Anti-windup clamp
    if (I_TERM > I_MAX) I_TERM = I_MAX;
    else if (I_TERM < -I_MAX) I_TERM = -I_MAX;
  }

  // D
  float derivativeRaw;
  if (DERIV_ON_ERROR) {
    static int lastErrorForDeriv = 0;
    derivativeRaw = (float)(error - lastErrorForDeriv);
    lastErrorForDeriv = error;
  } else {
    // derivative on measurement (position)
    if (lastPosition < 0) lastPosition = position;
    derivativeRaw = (float)(position - lastPosition);
    lastPosition = position;
  }
  // Filter derivative
  lastFilteredDerivative = (DERIVATIVE_FILTER * lastFilteredDerivative) +
                           ((1.0f - DERIVATIVE_FILTER) * derivativeRaw);
  float dTerm = Kd * lastFilteredDerivative;

  // Combine
  float control = pTerm + I_TERM + dTerm;

  // Convert to differential speeds around base
  int leftSpeed  = baseEff - (int)control;
  int rightSpeed = baseEff + (int)control;

  // Constrain
  leftSpeed  = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  motorSet(leftSpeed, rightSpeed);

  lastError = error;

  // Debug
  debugPrint(true, position, error, baseEff, pTerm, I_TERM, dTerm, norm, raw);
}

// ========================== END ==================================