/*
根據最小距離同步發送速度指令
*/
#include <Arduino.h>
#include "esp32-hal-ledc.h"  // Add this line for ESP32 LEDC functions

/*  
ESP32 NodeMCU 32S - Integrated Ultrasonic Sensor + Motor Control
3x HC-SR04 Ultrasonic Sensors + 3 Motors with PID Control

Distance-based speed control:
- Close distance (< 20cm): Speed = 0 mm/s
- Medium distance (20-40cm): Speed = 50 mm/s  
- Far distance (> 40cm): Speed = 100 mm/s

Manual commands still available:
s150     // Set all motors target speed to 150 mm/s
s1:100   // Set motor 1 target speed to 100 mm/s
kp1.2    // Set proportional gain
ki0.3    // Set integral gain
kd0.05   // Set derivative gain
*/

// --- Ultrasonic Sensor Pins ---
// Sensor 1
#define TRIG_PIN_1 15
#define ECHO_PIN_1 2

// Sensor 2
#define TRIG_PIN_2 0
#define ECHO_PIN_2 4

// Sensor 3
#define TRIG_PIN_3 22
#define ECHO_PIN_3 23

// --- Motor Pin Definitions (3 L298N modules) ---
// Motor 1 19~16
#define ENCODER_A1 19
#define ENCODER_B1 18
#define L298N_ENA_1 5
#define L298N_IN1_1 17
#define L298N_IN2_1 16

// Motor 2 34~25
#define ENCODER_B2 34
#define ENCODER_A2 35
#define L298N_ENA_2 32
#define L298N_IN2_2 33
#define L298N_IN1_2 25

// Motor 3 26~13
#define ENCODER_B3 26
#define ENCODER_A3 27
#define L298N_IN2_3 14
#define L298N_IN1_3 12
#define L298N_ENA_3 13

// --- PWM Settings (ESP32 specific) ---
#define PWM_FREQ 1000
#define PWM_RES 8  // 8-bit resolution: 0–255

// --- Ultrasonic Variables ---
long duration1, duration2, duration3;
int distance1, distance2, distance3;

// --- Encoder Variables (3 motors) ---
volatile long encoderCount[3] = {0, 0, 0};
long lastEncoderCount[3] = {0, 0, 0};
float currentSpeed[3] = {0, 0, 0};
float totalDistance_mm[3] = {0, 0, 0};

// --- PID Parameters (3 motors) ---
float Kp = 0.65;
float Ki = 1.0;
float Kd = 0.0;
float targetSpeed_mmps[3] = {0, 0, 0};  // Start with 0 speed
float integral[3] = {0, 0, 0};
float previousError[3] = {0, 0, 0};

// --- Control Parameters ---
const int PWM_MAX = 204; // About 80% of 255
bool manualMode = false;  // Flag for manual control mode

// --- Timing ---
unsigned long lastTime = 0;
unsigned long lastSensorTime = 0;

// --- Mechanical Parameters ---
const float wheel_circumference_mm = 219.91;
const int ticks_per_rev = 1320;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

// --- Distance Thresholds ---
const int CLOSE_DISTANCE = 20;   // cm
const int MEDIUM_DISTANCE = 40;  // cm
const int SPEED_CLOSE = 0;       // mm/s
const int SPEED_MEDIUM = 50;     // mm/s
const int SPEED_FAR = 100;       // mm/s

// --- Interrupt Service Functions ---
void IRAM_ATTR onEncoder1() {
  int A = digitalRead(ENCODER_A1);
  int B = digitalRead(ENCODER_B1);
  if (A == B) encoderCount[0]++;
  else encoderCount[0]--;
}

void IRAM_ATTR onEncoder2() {
  int A = digitalRead(ENCODER_A2);
  int B = digitalRead(ENCODER_B2);
  if (A == B) encoderCount[1]++;
  else encoderCount[1]--;
}

void IRAM_ATTR onEncoder3() {
  int A = digitalRead(ENCODER_A3);
  int B = digitalRead(ENCODER_B3);
  if (A == B) encoderCount[2]++;
  else encoderCount[2]--;
}

// --- Ultrasonic Sensor Reading Function ---
int readUltrasonic(int trigPin, int echoPin) {
  long duration;
  int distance;
  
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send 10 microsecond pulse to trigger pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo pin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance in cm
  distance = duration * 0.034 / 2;
  
  // Return distance (or -1 if reading failed)
  if (distance >= 400 || distance <= 2) {
    return -1; // Out of range
  }
  return distance;
}

// --- Speed Determination Based on Distance ---
int determineSpeed(int distance) {
  if (distance == -1) return SPEED_CLOSE;  // If sensor fails, stop
  
  if (distance < CLOSE_DISTANCE) {
    return SPEED_CLOSE;
  } else if (distance < MEDIUM_DISTANCE) {
    return SPEED_MEDIUM;
  } else {
    return SPEED_FAR;
  }
}

// --- Motor Control Function ---
void controlMotor(int motorIndex, float output) {
  int in1_pin, in2_pin, ena_pin;
  
  // Select corresponding pins
  switch(motorIndex) {
    case 0:
      in1_pin = L298N_IN1_1;
      in2_pin = L298N_IN2_1;
      ena_pin = L298N_ENA_1;
      break;
    case 1:
      in1_pin = L298N_IN1_2;
      in2_pin = L298N_IN2_2;
      ena_pin = L298N_ENA_2;
      break;
    case 2:
      in1_pin = L298N_IN1_3;
      in2_pin = L298N_IN2_3;
      ena_pin = L298N_ENA_3;
      break;
    default:
      return;
  }
  
  // Motor control
  if (output >= 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    ledcWrite(ena_pin, output);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    ledcWrite(ena_pin, -output);
  }
}

void setup() {
  Serial.begin(115200);
  
  // --- Ultrasonic Sensor Initialization ---
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);

  // --- Encoder Initialization ---
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), onEncoder1, CHANGE);

  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), onEncoder2, CHANGE);

  pinMode(ENCODER_A3, INPUT_PULLUP);
  pinMode(ENCODER_B3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A3), onEncoder3, CHANGE);

  // --- Motor Pin Initialization ---
  pinMode(L298N_IN1_1, OUTPUT);
  pinMode(L298N_IN2_1, OUTPUT);
  pinMode(L298N_IN1_2, OUTPUT);
  pinMode(L298N_IN2_2, OUTPUT);
  pinMode(L298N_IN1_3, OUTPUT);
  pinMode(L298N_IN2_3, OUTPUT);

  // --- PWM Initialization (ESP32 Core 3.0+ syntax) ---
  ledcAttach(L298N_ENA_1, PWM_FREQ, PWM_RES);  // Motor 1 PWM
  ledcAttach(L298N_ENA_2, PWM_FREQ, PWM_RES);  // Motor 2 PWM
  ledcAttach(L298N_ENA_3, PWM_FREQ, PWM_RES);  // Motor 3 PWM

  lastTime = millis();
  lastSensorTime = millis();
  
  Serial.println("ESP32 Integrated Ultrasonic + Motor Control System");
  Serial.println("==================================================");
  Serial.println("Auto mode: Motors controlled by 3 ultrasonic sensors");
  Serial.println("Distance < 20cm: Speed = 0 mm/s");
  Serial.println("Distance 20-40cm: Speed = 50 mm/s");
  Serial.println("Distance > 40cm: Speed = 100 mm/s");
  Serial.println();
  Serial.println("Manual commands available:");
  Serial.println("s150 (all motors), s1:100 (motor 1), s2:200 (motor 2), s3:150 (motor 3)");
  Serial.println("kp1.2/ki0.3/kd0.05 adjust PID parameters");
  Serial.println("auto - return to automatic mode");
  Serial.println();
  
  delay(2000);
}

void loop() {
  // === Process Serial Input ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.equals("auto")) {
      manualMode = false;
      Serial.println("Switched to automatic mode - sensors controlling motors");
    } else if (cmd.startsWith("s")) {
      manualMode = true;
      if (cmd.indexOf(":") > 0) {
        // Set individual motor: s1:100, s2:200, s3:150
        int motorNum = cmd.substring(1, cmd.indexOf(":")).toInt();
        float speed = cmd.substring(cmd.indexOf(":") + 1).toFloat();
        if (motorNum >= 1 && motorNum <= 3) {
          targetSpeed_mmps[motorNum - 1] = speed;
          Serial.print("Set motor "); Serial.print(motorNum); 
          Serial.print(" target speed to: "); Serial.println(speed);
        }
      } else {
        // Set all motors to same speed: s150
        float speed = cmd.substring(1).toFloat();
        for (int i = 0; i < 3; i++) {
          targetSpeed_mmps[i] = speed;
        }
        Serial.print("Set all motors target speed to: "); Serial.println(speed);
      }
    } else if (cmd.startsWith("kp")) {
      Kp = cmd.substring(2).toFloat();
      Serial.print("Kp = "); Serial.println(Kp);
    } else if (cmd.startsWith("ki")) {
      Ki = cmd.substring(2).toFloat();
      Serial.print("Ki = "); Serial.println(Ki);
    } else if (cmd.startsWith("kd")) {
      Kd = cmd.substring(2).toFloat();
      Serial.print("Kd = "); Serial.println(Kd);
    }
  }

  // === Read Ultrasonic Sensors (every 500ms) ===
  unsigned long now = millis();
  if (now - lastSensorTime >= 500) {
    // Read all three sensors
    distance1 = readUltrasonic(TRIG_PIN_1, ECHO_PIN_1);
    distance2 = readUltrasonic(TRIG_PIN_2, ECHO_PIN_2);
    distance3 = readUltrasonic(TRIG_PIN_3, ECHO_PIN_3);
    
    // If in automatic mode, determine target speeds based on sensor readings
    if (!manualMode) {
      // Use the minimum distance from all three sensors (most restrictive)
      int minDistance = distance1;
      
      // Check sensor 2
      if (distance2 != -1 && (minDistance == -1 || distance2 < minDistance)) {
        minDistance = distance2;
      }
      
      // Check sensor 3
      if (distance3 != -1 && (minDistance == -1 || distance3 < minDistance)) {
        minDistance = distance3;
      }
      
      // Determine target speed based on minimum distance
      int targetSpeed = determineSpeed(minDistance);
      
      // Set all motors to the same target speed
      for (int i = 0; i < 3; i++) {
        targetSpeed_mmps[i] = targetSpeed;
      }
    }
    
    lastSensorTime = now;
  }

  // === PID Control and Distance Update (every 100ms) ===
  if (now - lastTime >= 100) {
    
    // Process PID control for all 3 motors
    for (int i = 0; i < 3; i++) {
      long count = encoderCount[i];
      long tickDelta = count - lastEncoderCount[i];
      currentSpeed[i] = tickDelta * 10 * mm_per_tick;
      lastEncoderCount[i] = count;

      // PID Control
      float error = targetSpeed_mmps[i] - currentSpeed[i];
      integral[i] += error * 0.1;
      float derivative = (error - previousError[i]) / 0.1;
      float output = Kp * error + Ki * integral[i] + Kd * derivative;
      previousError[i] = error;

      output = constrain(output, -PWM_MAX, PWM_MAX); // Limit output

      // Control motor
      controlMotor(i, output);

      // Update cumulative distance
      totalDistance_mm[i] = encoderCount[i] * mm_per_tick;
    }

    // Display to Serial Plotter and Monitor
    Serial.print("Dist1:"); Serial.println(distance1);
    Serial.print(",Dist2:"); Serial.println(distance2);
    Serial.print(",Dist3:"); Serial.println(distance3);
    /*Serial.print(",Target1:"); Serial.print(targetSpeed_mmps[0]);
    Serial.print(",Speed1:"); Serial.print(currentSpeed[0]);
    Serial.print(",Target2:"); Serial.print(targetSpeed_mmps[1]);
    Serial.print(",Speed2:"); Serial.print(currentSpeed[1]);
    Serial.print(",Target3:"); Serial.print(targetSpeed_mmps[2]);
    Serial.print(",Speed3:"); Serial.println(currentSpeed[2]);
    */
    lastTime = now;
  }
}