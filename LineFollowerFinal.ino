/*
  High-Precision PID Line Follower (4-motor variant, simplified)
  - Uses AFMotor L293D shield (AFMotor library)
  - Sensors: A0..A4 (5 sensors)
  - Motor mapping:
      Right side: M1 and M3
      Left  side: M2 and M4
  - Pure PID control (no CLP, no NEAR obstacle logic)
*/

#include <AFMotor.h>

// ========================== USER CONFIG ==========================

// --- Sensor & Pins ---
const uint8_t NUM_SENSORS = 5;
uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4}; // index 0 MUST be left-most physically

// --- Motor Setup (four motors) ---
AF_DCMotor motor1(1); // Right (front-right)
AF_DCMotor motor2(2); // Left  (front-left)
AF_DCMotor motor3(3); // Right (rear-right)
AF_DCMotor motor4(4); // Left  (rear-left)

// If a motor runs backwards when you command forward, flip its direction constant:
uint8_t LEFT_DIR_DEFAULT  = FORWARD;  // applied to motors 2 & 4
uint8_t RIGHT_DIR_DEFAULT = FORWARD;  // applied to motors 1 & 3

// --- Polarity ---
bool ENABLE_AUTO_POLARITY = false;
bool BLACK_LOW = true;             // if black reflectance -> lower raw value
bool REVERSE_ERROR = false;        // if turning direction is inverted, set true

// --- PID Gains ---
float Kp = 0.13f;
float Ki = 0.00055f;
float Kd = 2.2f;

float DERIVATIVE_FILTER = 0.6f;
bool DERIV_ON_ERROR = true;

// Integral settings
float I_TERM = 0.0f;
float I_MAX = 300.0f;
bool INTEGRAL_WHEN_LOST = false;

// --- Speeds ---
int BASE_SPEED = 40;
int MAX_SPEED  = 120;
int MIN_BASE_SPEED = 90;
int STARTUP_RAMP_TIME_MS = 2000;
bool USE_STARTUP_RAMP = true;

// Adaptive base speed
bool ENABLE_ADAPTIVE_BASE = true;
int ERROR_FOR_MIN_BASE = 1400;

// --- Smoothing ---
const uint8_t SMOOTH_SAMPLES = 4;
int rawHistory[NUM_SENSORS][SMOOTH_SAMPLES];
uint8_t historyIndex = 0;

// --- Calibration ---
unsigned long CALIBRATION_TIME_MS = 3000;

// --- Lost Line Handling ---
unsigned long lastLineMillis = 0;
unsigned long LOST_LINE_GRACE_MS = 250;
unsigned long LOST_LINE_SEARCH_MS = 800;
int LOST_LINE_SEARCH_SPEED = 110;
bool lastLineDetected = true;
int lastError = 0;

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

  motor2.setSpeed(leftSpeed);
  motor4.setSpeed(leftSpeed);
  motor2.run(LEFT_DIR);
  motor4.run(LEFT_DIR);

  motor1.setSpeed(rightSpeed);
  motor3.setSpeed(rightSpeed);
  motor1.run(RIGHT_DIR);
  motor3.run(RIGHT_DIR);
}

void motorStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void readRaw(int rawVals[]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    rawVals[i] = analogRead(sensorPins[i]);
  }
}

void updateHistory(const int rawVals[]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    rawHistory[i][historyIndex] = rawVals[i];
  }
  historyIndex = (historyIndex + 1) % SMOOTH_SAMPLES;
}

void getSmoothedRaw(int out[]) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    long sum = 0;
    for (uint8_t k = 0; k < SMOOTH_SAMPLES; k++)
      sum += rawHistory[i][k];
    out[i] = sum / SMOOTH_SAMPLES;
  }
}

void readNormalized(int normVals[]) {
  int smoothed[NUM_SENSORS];
  getSmoothedRaw(smoothed);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int span = sensorMax[i] - sensorMin[i];
    if (span < 5) span = 5;
    long v = (long)(smoothed[i] - sensorMin[i]) * 1000L / span;
    if (v < 0) v = 0;
    if (v > 1000) v = 1000;

    if (BLACK_LOW) v = 1000 - v;
    normVals[i] = (int)v;
  }
}

long computePosition(const int normVals[], bool &found) {
  long numerator = 0;
  long denom = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    int weight = normVals[i];
    if (weight > 40) {
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

int computeAdaptiveBase(int error) {
  if (!ENABLE_ADAPTIVE_BASE) return BASE_SPEED;
  int aerr = abs(error);
  if (aerr >= ERROR_FOR_MIN_BASE) return MIN_BASE_SPEED;
  float f = (float)aerr / ERROR_FOR_MIN_BASE;
  float v = (float)BASE_SPEED - ((float)(BASE_SPEED - MIN_BASE_SPEED) * f);
  return (int)v;
}

int applyStartupRamp(int base) {
  if (!USE_STARTUP_RAMP) return base;
  unsigned long elapsed = millis() - startMillis;
  if (elapsed >= (unsigned long)STARTUP_RAMP_TIME_MS) return base;
  float ratio = (float)elapsed / (float)STARTUP_RAMP_TIME_MS;
  int minStart = min(base, 20);
  int ramped = minStart + (int)((base - minStart) * ratio);
  return ramped;
}

void lostLineHandler() {
  unsigned long noLineTime = millis() - lastLineMillis;
  if (noLineTime < LOST_LINE_GRACE_MS) {
    motorSet(80, 80);
  } else if (noLineTime < LOST_LINE_GRACE_MS + LOST_LINE_SEARCH_MS) {
    if (lastError >= 0) {
      motorSet(LOST_LINE_SEARCH_SPEED, 0);
    } else {
      motorSet(0, LOST_LINE_SEARCH_SPEED);
    }
  } else {
    motorStop();
  }
}

void autoPolaritySequence() {
  Serial.println(F("AUTO POLARITY: Place center sensor over WHITE..."));
  delay(1500);
  int whiteVal = analogRead(sensorPins[NUM_SENSORS / 2]);
  Serial.println(whiteVal);
  Serial.println(F("Now over BLACK..."));
  delay(2000);
  int blackVal = analogRead(sensorPins[NUM_SENSORS / 2]);
  Serial.println(blackVal);

  if (blackVal < whiteVal) {
    BLACK_LOW = true;
  } else {
    BLACK_LOW = false;
  }
}

void calibrateSensors() {
  Serial.println(F("Calibrating..."));
  unsigned long t0 = millis();
  while (millis() - t0 < CALIBRATION_TIME_MS) {
    int raw[NUM_SENSORS];
    readRaw(raw);
    updateHistory(raw);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (raw[i] < sensorMin[i]) sensorMin[i] = raw[i];
      if (raw[i] > sensorMax[i]) sensorMax[i] = raw[i];
    }
    delay(5);
  }
  Serial.println(F("Calibration done."));
}

void debugPrint(bool lineFound, long position, int error, int baseEff,
                float pTerm, float iTerm, float dTerm,
                const int normVals[], const int rawVals[]) {
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
  Serial.println();
}

// ========================== SETUP ================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n--- PID Line Follower Start (4-motor) ---"));

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
}

// ========================== LOOP =================================
void loop() {
  int raw[NUM_SENSORS];
  readRaw(raw);
  updateHistory(raw);

  int norm[NUM_SENSORS];
  readNormalized(norm);

  bool lineFound;
  long position = computePosition(norm, lineFound);

  if (!lineFound) {
    lastLineDetected = false;
    if (!INTEGRAL_WHEN_LOST) {}
    lostLineHandler();
    debugPrint(false, position, lastError, BASE_SPEED, 0, I_TERM, 0, norm, raw);
    return;
  }

  if (!lastLineDetected && lineFound) {
    I_TERM *= 0.5f;
  }
  lastLineDetected = true;
  lastLineMillis = millis();

  long center = (long)(NUM_SENSORS - 1) * 1000L / 2L;
  int error = (int)(position - center);
  if (REVERSE_ERROR) error = -error;

  int baseEff = computeAdaptiveBase(error);
  baseEff = applyStartupRamp(baseEff);

  float pTerm = Kp * (float)error;
  if (lineFound && (abs(error) < 2500)) {
    I_TERM += Ki * error;
    if (I_TERM > I_MAX) I_TERM = I_MAX;
    else if (I_TERM < -I_MAX) I_TERM = -I_MAX;
  }

  float derivativeRaw;
  if (DERIV_ON_ERROR) {
    static int lastErrorForDeriv = 0;
    derivativeRaw = (float)(error - lastErrorForDeriv);
    lastErrorForDeriv = error;
  } else {
    if (lastPosition < 0) lastPosition = position;
    derivativeRaw = (float)(position - lastPosition);
    lastPosition = position;
  }

  lastFilteredDerivative = (DERIVATIVE_FILTER * lastFilteredDerivative) +
                           ((1.0f - DERIVATIVE_FILTER) * derivativeRaw);
  float dTerm = Kd * lastFilteredDerivative;

  float control = pTerm + I_TERM + dTerm;

  int leftSpeed  = baseEff - (int)control;
  int rightSpeed = baseEff + (int)control;

  leftSpeed  = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  motorSet(leftSpeed, rightSpeed);

  lastError = error;

  debugPrint(true, position, error, baseEff, pTerm, I_TERM, dTerm, norm, raw);
}
