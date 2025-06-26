// motor_control.ino - ESP32 Basic Mecanum Wheel Motor Control
#include <Arduino.h>

// Motor pin assignments (example)
const int motor1PWM = 5;
const int motor1DIR = 18;
const int motor2PWM = 17;
const int motor2DIR = 16;
const int motor3PWM = 4;
const int motor3DIR = 0;

void setup() {
  Serial.begin(115200);
  
  // Set motor control pins as outputs
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1DIR, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2DIR, OUTPUT);
  pinMode(motor3PWM, OUTPUT);
  pinMode(motor3DIR, OUTPUT);
  
  Serial.println("Motor control ready.");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'f') {
      moveForward();
    }
    if (command == 'b') {
      moveBackward();
    }
    if (command == 's') {
      stopMotors();
    }
  }
}

void moveForward() {
  analogWrite(motor1PWM, 200);
  digitalWrite(motor1DIR, HIGH);
  analogWrite(motor2PWM, 200);
  digitalWrite(motor2DIR, HIGH);
  analogWrite(motor3PWM, 200);
  digitalWrite(motor3DIR, HIGH);
}

void moveBackward() {
  analogWrite(motor1PWM, 200);
  digitalWrite(motor1DIR, LOW);
  analogWrite(motor2PWM, 200);
  digitalWrite(motor2DIR, LOW);
  analogWrite(motor3PWM, 200);
  digitalWrite(motor3DIR, LOW);
}

void stopMotors() {
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  analogWrite(motor3PWM, 0);
}
