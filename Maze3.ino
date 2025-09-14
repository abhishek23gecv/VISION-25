#include <AFMotor.h>

// ---------------- Motors (Adafruit Motor Shield v1) ----------------
AF_DCMotor mR1(1); // right motor 1
AF_DCMotor mL1(2); // left  motor 1
AF_DCMotor mR2(3); // right motor 2
AF_DCMotor mL2(4); // left  motor 2

// ---------------- Sensor pins ----------------
const int trigL = A0;
const int echoL = A1;
const int trigF = A2;
const int echoF = A3;

// ---------------- TUNING ----------------
const int BASE_SPEED   = 110;
const int BACK_SPEED   = 110;
const int MAX_SPEED    = 170;
const int DESIRED_LEFT = 15;   // ideal distance from left wall
const int FRONT_LIMIT  = 14;   // stop + turn if wall ahead
const float KP         = 5.0;  // proportional gain
const int LEFT_MARGIN  = 4;    // tolerance band around DESIRED_LEFT

// ---------------- Misc ----------------
const int US_SAMPLES = 5;
const unsigned long PULSE_TIMEOUT = 20000UL;

// ---------------- Motor helpers ----------------
void drive(int leftSigned, int rightSigned) {
  int leftSpd  = constrain(abs(leftSigned), 0, MAX_SPEED);
  int rightSpd = constrain(abs(rightSigned), 0, MAX_SPEED);

  mL1.setSpeed(leftSpd); mL2.setSpeed(leftSpd);
  mR1.setSpeed(rightSpd); mR2.setSpeed(rightSpd);

  if (leftSigned >= 0) { mL1.run(FORWARD); mL2.run(FORWARD); }
  else                 { mL1.run(BACKWARD); mL2.run(BACKWARD); }

  if (rightSigned >= 0){ mR1.run(FORWARD); mR2.run(FORWARD); }
  else                 { mR1.run(BACKWARD); mR2.run(BACKWARD); }
}

void stopAll() {
  mL1.run(RELEASE); mL2.run(RELEASE);
  mR1.run(RELEASE); mR2.run(RELEASE);
}

void pivotRightMs(unsigned long ms) {
  int tSpeed = MAX_SPEED * 0.85;
  mL1.setSpeed(tSpeed); mL2.setSpeed(tSpeed);
  mR1.setSpeed(tSpeed); mR2.setSpeed(tSpeed);
  mL1.run(FORWARD); mL2.run(FORWARD);
  mR1.run(BACKWARD); mR2.run(BACKWARD);
  delay(ms);
  stopAll();
}

void backupMs(unsigned long t) {
  drive(-BACK_SPEED, -BACK_SPEED);
  delay(t);
  stopAll();
}

// ---------------- Ultrasonic helpers ----------------
long readUSonce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long dur = pulseIn(echoPin, HIGH, PULSE_TIMEOUT);
  if (dur == 0) return 500;
  return (long)(dur * 0.034 / 2.0);
}

long readUS_median(int trigPin, int echoPin, int samples = US_SAMPLES) {
  if (samples < 1) samples = 1;
  if (samples > 11) samples = 11;
  long vals[11];
  for (int i = 0; i < samples; i++) {
    vals[i] = readUSonce(trigPin, echoPin);
    delay(6);
  }
  for (int i = 1; i < samples; i++) {
    long key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) {
      vals[j + 1] = vals[j];
      j--;
    }
    vals[j + 1] = key;
  }
  return vals[samples / 2];
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(trigL, OUTPUT); digitalWrite(trigL, LOW);
  pinMode(echoL, INPUT);
  pinMode(trigF, OUTPUT); digitalWrite(trigF, LOW);
  pinMode(echoF, INPUT);
  stopAll();
  delay(200);
  Serial.println("Maze follower startup");
}

// ---------------- Main loop ----------------
void loop() {
  long dFront = readUS_median(trigF, echoF, 5);
  delay(8);
  long dLeft  = readUS_median(trigL, echoL, 5);

  Serial.print("Front: "); Serial.print(dFront);
  Serial.print(" cm  | Left: "); Serial.println(dLeft);

  // --- Handle front obstacle ---
  if (dFront <= FRONT_LIMIT) {
    stopAll(); delay(40);
    backupMs(260); delay(40);
    pivotRightMs(350); delay(60);
    stopAll(); delay(80);
    return;
  }

  // --- Handle left wall too close (emergency) ---
  if (dLeft < 6) {
    Serial.println("!! Emergency: very close to left wall -> quick right pivot");
    pivotRightMs(150);
    stopAll(); delay(40);
    return;
  }

  // --- Handle left wall within tolerance ---
  int error = dLeft - DESIRED_LEFT;

  // NEW: If wall much closer than desired (but not emergency), steer gently right
  if (error < -LEFT_MARGIN) {
    Serial.println("Left wall too close (soft correction)");
    drive(BASE_SPEED + 20, BASE_SPEED - 20); // nudge right
    delay(30);
    return;
  }

  // Use proportional control otherwise
  float corr = KP * error;
  int leftSpeed  = constrain((int)(BASE_SPEED - corr), 60, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED + corr), 60, MAX_SPEED);

  drive(leftSpeed, rightSpeed);
  delay(30);
}
