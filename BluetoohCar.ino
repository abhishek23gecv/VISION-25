#include <AFMotor.h>
#include <SoftwareSerial.h>

// Motors
AF_DCMotor motorRight1(1);
AF_DCMotor motorLeft1(2);
AF_DCMotor motorRight2(3);
AF_DCMotor motorLeft2(4);

// Bluetooth on pins 0 (RX) and 1 (TX)
SoftwareSerial serialBT(0, 1);

char BT;       // received command
int Speed = 150;  // default speed

void setup() {
  serialBT.begin(9600);   // HC-05 default baud
  setMotorSpeed(Speed);
}

void loop() {
  while (serialBT.available()) {
    BT = serialBT.read();

    // --- Speed control ---
    if (BT == '0') Speed = 100;
    if (BT == '1') Speed = 110;
    if (BT == '2') Speed = 120;
    if (BT == '3') Speed = 130;
    if (BT == '4') Speed = 140;
    if (BT == '5') Speed = 150;
    if (BT == '6') Speed = 180;
    if (BT == '7') Speed = 200;
    if (BT == '8') Speed = 220;
    if (BT == '9') Speed = 240;
    if (BT == 'q') Speed = 255;

    // --- Movement control ---
    if (BT == 'F') go_forward();
    else if (BT == 'B') go_backward();
    else if (BT == 'L') go_left();
    else if (BT == 'R') go_right();
    else if (BT == 'S') stopMotors();
    else if (BT == 'I') forward_right();
    else if (BT == 'J') backward_right();
    else if (BT == 'G') forward_left();
    else if (BT == 'H') backward_left();
  }
}

// ------------------ Motor helpers ------------------
void setMotorSpeed(int spd) {
  motorRight1.setSpeed(spd);
  motorRight2.setSpeed(spd);
  motorLeft1.setSpeed(spd);
  motorLeft2.setSpeed(spd);
}

void go_forward() {
  setMotorSpeed(Speed);
  motorRight1.run(FORWARD);
  motorRight2.run(FORWARD);
  motorLeft1.run(FORWARD);
  motorLeft2.run(FORWARD);
}

void go_backward() {
  setMotorSpeed(Speed);
  motorRight1.run(BACKWARD);
  motorRight2.run(BACKWARD);
  motorLeft1.run(BACKWARD);
  motorLeft2.run(BACKWARD);
}

void go_left() {
  motorRight1.setSpeed(Speed);
  motorRight2.setSpeed(Speed);
  motorLeft1.setSpeed(Speed / 2);
  motorLeft2.setSpeed(Speed / 2);

  motorRight1.run(FORWARD);
  motorRight2.run(FORWARD);
  motorLeft1.run(BACKWARD);
  motorLeft2.run(BACKWARD);
}

void go_right() {
  motorLeft1.setSpeed(Speed);
  motorLeft2.setSpeed(Speed);
  motorRight1.setSpeed(Speed / 2);
  motorRight2.setSpeed(Speed / 2);

  motorLeft1.run(FORWARD);
  motorLeft2.run(FORWARD);
  motorRight1.run(BACKWARD);
  motorRight2.run(BACKWARD);
}

void forward_left() {
  motorRight1.setSpeed(Speed);
  motorRight2.setSpeed(Speed);
  motorLeft1.setSpeed(Speed / 2);
  motorLeft2.setSpeed(Speed / 2);

  motorRight1.run(FORWARD);
  motorRight2.run(FORWARD);
  motorLeft1.run(FORWARD);
  motorLeft2.run(FORWARD);
}

void forward_right() {
  motorLeft1.setSpeed(Speed);
  motorLeft2.setSpeed(Speed);
  motorRight1.setSpeed(Speed / 2);
  motorRight2.setSpeed(Speed / 2);

  motorLeft1.run(FORWARD);
  motorLeft2.run(FORWARD);
  motorRight1.run(FORWARD);
  motorRight2.run(FORWARD);
}

void backward_left() {
  motorRight1.setSpeed(Speed);
  motorRight2.setSpeed(Speed);
  motorLeft1.setSpeed(Speed / 2);
  motorLeft2.setSpeed(Speed / 2);

  motorRight1.run(BACKWARD);
  motorRight2.run(BACKWARD);
  motorLeft1.run(BACKWARD);
  motorLeft2.run(BACKWARD);
}

void backward_right() {
  motorLeft1.setSpeed(Speed);
  motorLeft2.setSpeed(Speed);
  motorRight1.setSpeed(Speed / 2);
  motorRight2.setSpeed(Speed / 2);

  motorLeft1.run(BACKWARD);
  motorLeft2.run(BACKWARD);
  motorRight1.run(BACKWARD);
  motorRight2.run(BACKWARD);
}

void stopMotors() {
  motorRight1.run(RELEASE);
  motorRight2.run(RELEASE);
  motorLeft1.run(RELEASE);
  motorLeft2.run(RELEASE);
}