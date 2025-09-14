// === Bluetooth RC Car for Arduino UNO + HC-05/06 ===
// L298N Motor Driver connections
const int ENA = 5;    // PWM pin for Left motors  (Enable A)
const int ENB = 6;    // PWM pin for Right motors (Enable B)
const int IN1 = 8;    // Left motor direction 1
const int IN2 = 9;    // Left motor direction 2
const int IN3 = 10;   // Right motor direction 1
const int IN4 = 11;   // Right motor direction 2

char BT;
int Speed = 255;

void setup() {
  Serial.begin(9600);          // match your HC-05/06 baud rate (default 9600)
  
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stop();                      // make sure motors are off
  Serial.println("UNO Bluetooth RC Car Ready!");
}

void loop() {
  if (Serial.available()) {
    BT = Serial.read();

    // ----- Speed control -----
    if (BT >= '0' && BT <= '9') Speed = map(BT - '0', 0, 9, 255, 950);
    if (BT == 'q') Speed = 255;

    // ----- Movement commands -----
    switch (BT) {
      case 'F': go_forward(); break;
      case 'B': go_backward(); break;
      case 'R': go_left();    break;
      case 'L': go_right();   break;
      case 'S': stop();       break;
      case 'I': forward_right();  break;
      case 'J': backward_right(); break;
      case 'G': forward_left();   break;
      case 'H': backward_left();  break;
    }

    Serial.print("Cmd: ");
    Serial.print(BT);
    Serial.print(" | Speed: ");
    Serial.println(Speed);
  }
}

// ---------- Motor Functions ----------
void go_forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed); analogWrite(ENB, Speed);
}

void go_backward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed); analogWrite(ENB, Speed);
}

void go_left() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed); analogWrite(ENB, Speed);
}

void go_right() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed); analogWrite(ENB, Speed);
}

void stop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);    analogWrite(ENB, 0);
}

void forward_right() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed); analogWrite(ENB, Speed / 2);
}

void backward_right() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed); analogWrite(ENB, Speed / 2);
}

void forward_left() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, Speed / 2); analogWrite(ENB, Speed);
}

void backward_left() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, Speed / 2); analogWrite(ENB, Speed);
}