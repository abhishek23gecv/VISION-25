#include <AFMotor.h>

// ================== Motor Setup ==================
AF_DCMotor mR1(1);
AF_DCMotor mR2(3);
AF_DCMotor mL1(2);
AF_DCMotor mL2(4);

// Base speeds
int BASE_SPEED = 120;
int MIN_DRIVE_SPEED = 70;
int MAX_DRIVE_SPEED = 200;
int TURN_SPEED = 90;
int TURN_SPEED_SLOW = 70;

// Turn timing (ms) – must calibrate
int TURN_TIME_90 = 310;      // left/right 90°
int TURN_BRAKE_PAUSE = 80;

// ================== Ultrasonic Pins ==================
const int trigL = A0;
const int echoL = A1;
const int trigF = A2;
const int echoF = A3;

// ================== Maze / Distance Parameters ==================
const float TARGET_LEFT = 5.5;          // Desired distance to left wall (cm)
const float LEFT_OPEN_THRESHOLD = 14.0; // If left distance > this => junction/opening
const float FRONT_BLOCK_THRESHOLD = 10.0; // If front < this => blocked
const float FRONT_CLEAR_FOR_MOVE = 14.0;  // Need at least this to proceed forward after a turn
const float MIN_VALID_CM = 2.0;         // Reject unrealistically small echoes
const float MAX_VALID_CM = 250.0;       // Cap for ultrasonic sanity

// P-control gain for wall following
float KP = 8.0; // Adjust (5–10 typical)

// Safety / logic timing
unsigned long lastDecisionMillis = 0;
const unsigned long DECISION_INTERVAL = 70; // ms between major decisions

// Debug flags
bool DEBUG_DISTANCE = false;
bool DEBUG_DECISIONS = false;

// ================== Utility: Motor Helpers ==================
void setSideSpeeds(int leftSpd, int rightSpd) {
  leftSpd  = constrain(leftSpd, 0, 255);
  rightSpd = constrain(rightSpd, 0, 255);
  mL1.setSpeed(leftSpd); mL2.setSpeed(leftSpd);
  mR1.setSpeed(rightSpd); mR2.setSpeed(rightSpd);
}

void driveForward(int leftSpd, int rightSpd) {
  setSideSpeeds(leftSpd, rightSpd);
  mL1.run(FORWARD); mL2.run(FORWARD);
  mR1.run(FORWARD); mR2.run(FORWARD);
}

void stopAll() {
  mL1.run(RELEASE); mL2.run(RELEASE);
  mR1.run(RELEASE); mR2.run(RELEASE);
}

void pivotLeft(bool precise = true) {
  int spd = precise ? TURN_SPEED_SLOW : TURN_SPEED;
  setSideSpeeds(spd, spd);
  mL1.run(BACKWARD); mL2.run(BACKWARD);
  mR1.run(FORWARD);  mR2.run(FORWARD);
}

void pivotRight(bool precise = true) {
  int spd = precise ? TURN_SPEED_SLOW : TURN_SPEED;
  setSideSpeeds(spd, spd);
  mL1.run(FORWARD);  mL2.run(FORWARD);
  mR1.run(BACKWARD); mR2.run(BACKWARD);
}

void brakePause() {
  stopAll();
  delay(TURN_BRAKE_PAUSE);
}

// ================== Ultrasonic Reading (Median Filter) ==================
long singlePulseCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, 25000UL); // timeout ~25ms
  if (dur == 0) return MAX_VALID_CM; // treat timeout as "far"
  float cm = dur * 0.0343 / 2.0;
  if (cm < MIN_VALID_CM) cm = MIN_VALID_CM;
  if (cm > MAX_VALID_CM) cm = MAX_VALID_CM;
  return (long)cm;
}

long median5(long a[]) {
  // Simple insertion sort for 5 elements
  for (int i=1;i<5;i++){
    long key=a[i];
    int j=i-1;
    while (j>=0 && a[j]>key) { a[j+1]=a[j]; j--; }
    a[j+1]=key;
  }
  return a[2];
}

long readDistanceMedian(int trig, int echo) {
  long vals[5];
  for (int i=0;i<5;i++) {
    vals[i] = singlePulseCM(trig, echo);
    delayMicroseconds(800);
  }
  return median5(vals);
}

// ================== Turning Helpers ==================
void doNinetyLeft() {
  if (DEBUG_DECISIONS) Serial.println(F("Action: Turn LEFT 90°"));
  pivotLeft(true);
  delay(TURN_TIME_90);
  brakePause();
}

void doNinetyRight() {
  if (DEBUG_DECISIONS) Serial.println(F("Action: Turn RIGHT 90°"));
  pivotRight(true);
  delay(TURN_TIME_90);
  brakePause();
}

// Turn right (or continue to 180) until front is clear enough
void pivotRightUntilFrontClear() {
  if (DEBUG_DECISIONS) Serial.println(F("Action: Right / U-turn sweep"));
  unsigned long start = millis();
  unsigned long maxDuration = 2500; // Safety cap (ms)
  while (millis() - start < maxDuration) {
    long dF = readDistanceMedian(trigF, echoF);
    if (dF > FRONT_CLEAR_FOR_MOVE) {
      brakePause();
      return;
    }
    pivotRight(false);
  }
  brakePause();
}

// ================== Wall Following Forward Drive ==================
void forwardWallFollow(long dLeft) {
  // P-control (error: want to be TARGET_LEFT from wall)
  float error = TARGET_LEFT - (float)dLeft;
  float adj = KP * error;

  int rightSpd = BASE_SPEED + (int)adj;
  int leftSpd  = BASE_SPEED - (int)adj;

  rightSpd = constrain(rightSpd, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
  leftSpd  = constrain(leftSpd,  MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);

  driveForward(leftSpd, rightSpd);
}

// ================== Setup ==================
void setup() {
  Serial.begin(9600);

  pinMode(trigL, OUTPUT); pinMode(echoL, INPUT);
  pinMode(trigF, OUTPUT); pinMode(echoF, INPUT);

  stopAll();

  Serial.println(F("Left-Wall Maze Solver Starting..."));
}

// ================== Main Loop ==================
void loop() {
  // Limit decision frequency
  if (millis() - lastDecisionMillis < DECISION_INTERVAL) {
    return;
  }
  lastDecisionMillis = millis();

  long dLeft  = readDistanceMedian(trigL, echoL);
  long dFront = readDistanceMedian(trigF, echoF);

  if (DEBUG_DISTANCE) {
    Serial.print(F("L:")); Serial.print(dLeft);
    Serial.print(F("  F:")); Serial.println(dFront);
  }

  bool leftOpen     = dLeft  > LEFT_OPEN_THRESHOLD;
  bool frontBlocked = dFront < FRONT_BLOCK_THRESHOLD;

  // ---- Decision Logic (Left-Hand Rule) ----
  if (leftOpen) {
    // Prefer LEFT turn
    doNinetyLeft();

    // After turn, optionally nudge forward if front is clear
    dFront = readDistanceMedian(trigF, echoF);
    if (dFront > FRONT_CLEAR_FOR_MOVE) {
      forwardWallFollow(readDistanceMedian(trigL, echoL));
    }
  }
  else if (frontBlocked) {
    // We must turn right (or 180 if dead-end)
    pivotRightUntilFrontClear();

    // After clearing pivot, go forward again (will wall-follow next cycle)
    dLeft = readDistanceMedian(trigL, echoL);
    forwardWallFollow(dLeft);
  }
  else {
    // Corridor ahead, follow wall
    forwardWallFollow(dLeft);
  }
}