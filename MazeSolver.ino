#include <AFMotor.h>

// ---------- Motors ----------
AF_DCMotor mR1(1);
AF_DCMotor mL1(2);
AF_DCMotor mR2(3);
AF_DCMotor mL2(4);

int fastSpeed = 120;   // straight speed
int turnSpeed = 60;   // precise turn speed

void setAll(int spd) {
  mR1.setSpeed(spd); 
  mR2.setSpeed(spd);
  mL1.setSpeed(spd); 
  mL2.setSpeed(spd);
}

void stopAll() {
  mR1.run(RELEASE); 
  mR2.run(RELEASE);
  mL1.run(RELEASE); 
  mL2.run(RELEASE);
}

void goForward(int spd) {
  setAll(spd);
  mR1.run(FORWARD); 
  mR2.run(FORWARD);
  mL1.run(FORWARD); 
  mL2.run(FORWARD);
}

void turnLeft() {
  setAll(turnSpeed);
  mR1.run(FORWARD); 
  mR2.run(FORWARD);
  mL1.run(BACKWARD); 
  mL2.run(BACKWARD);
  delay(300);        // ≈90°; tune to your chassis
  stopAll();
}

void turnRight() {
  setAll(turnSpeed);
  mL1.run(FORWARD); 
  mL2.run(FORWARD);
  mR1.run(BACKWARD); 
  mR2.run(BACKWARD);
  delay(300);
  stopAll();
}

// ---------- Ultrasonic ----------
const int trigL  = A0;
const int echoL  = A1;
const int trigF  = A2;
const int echoF  = A3;

const int wallDist = 12; // cm

long readUS(int trig, int echo) {
  digitalWrite(trig, LOW);  
  delayMicroseconds(2);
  digitalWrite(trig, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH) * 0.034 / 2;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigL, OUTPUT); 
  pinMode(echoL, INPUT);
  pinMode(trigF, OUTPUT); 
  pinMode(echoF, INPUT);
}

void loop() {
  long dLeft  = readUS(trigL, echoL);
  long dFront = readUS(trigF, echoF);

  // ---- Left-wall rule ----
  if (dLeft > wallDist) {
    // Left clear -> prefer left turn
    turnLeft();
    goForward(fastSpeed);
  }
  else if (dFront < wallDist) {
    // Front blocked -> right turn
    turnRight();
  }
  else {
    // Straight path
    goForward(fastSpeed);
  }

  delay(30); // quick sensor refresh
}