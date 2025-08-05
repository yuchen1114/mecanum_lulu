#include <Arduino.h>
#include "esp32-hal-ledc.h"

/*  
Motor controller with ultrasonic sensor for crash prevention
UART2 on GPIO16(RX), GPIO17(TX) @ 115200 baud
Ultrasonic sensor on GPIO26(TRIG), GPIO27(ECHO)

Command format: 
"vel1,vel2,vel3\n"  // Direct motor velocities in mm/s
Example: "100,150,200\n" sets motor1=100mm/s, motor2=150mm/s, motor3=200mm/s

Legacy commands for PID testing:
"s150\n"     // Set all motors to same speed
"s1:100\n"   // Set motor 1 speed
"kp1.2\n"    // Adjust PID parameters

Updated ultrasonic logic:
- Emergency stop when object detected within 20cm
- After 2 seconds in emergency stop, motors become active again
- Emergency stop flag resets when object moves out of 20cm range
*/

// --- Pin definitions for three L298N modules ---
// Motor 1
#define ENCODER_A1 15
#define ENCODER_B1 2
#define L298N_ENA_1 5
#define L298N_IN1_1 19
#define L298N_IN2_1 18

// Motor 2
#define ENCODER_A2 35
#define ENCODER_B2 34
#define L298N_ENA_2 32
#define L298N_IN1_2 25
#define L298N_IN2_2 33

// Motor 3
#define ENCODER_A3 22
#define ENCODER_B3 23
#define L298N_ENA_3 13
#define L298N_IN1_3 21
#define L298N_IN2_3 14

// --- Ultrasonic sensor pins ---
#define TRIG_PIN 26
#define ECHO_PIN 27

// --- PWM Settings (ESP32) ---
#define PWM_FREQ 50000
#define PWM_RES 8

// --- Safety settings ---
const float STOP_DISTANCE_CM = 20.0;  // Stop if object is closer than 20cm
const unsigned long ULTRASONIC_PERIOD = 50;  // Check distance every 50ms
const unsigned long EMERGENCY_STOP_DURATION = 2000;  // 2 seconds

// --- Encoder variables ---
volatile long encoderCount[3] = {0, 0, 0};
long lastEncoderCount[3] = {0, 0, 0};
float currentSpeed[3] = {0, 0, 0};

// --- PID parameters ---
float Kp = 0.65;
float Ki = 1.0;
float Kd = 0.0;
float targetSpeed_mmps[3] = {0, 0, 0};
float integral[3] = {0, 0, 0};
float previousError[3] = {0, 0, 0};

// --- Limits ---
const int PWM_MAX = 204;

// --- Timing ---
unsigned long lastTime = 0;
unsigned long lastUltrasonicTime = 0;
unsigned long emergencyStopStartTime = 0;

// --- Mechanical parameters ---
const float wheel_circumference_mm = 219.91;
const int ticks_per_rev = 1320;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

// --- Safety flags ---
bool emergencyStop = false;
bool objectDetected = false;

// --- Interrupt service routines ---
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

float getUltrasonicDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  
  // Calculate distance in cm
  if (duration == 0) {
    return 999.0; // No object detected
  }
  
  float distance = duration * 0.034 / 2.0;
  return distance;
}

void setup() {
  Serial.begin(115200);  // Debug serial
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // UART2 for receiving commands
  
  Serial.println("Motor controller with ultrasonic sensor ready");
  Serial.println("UART2 on GPIO16(RX), GPIO17(TX)");
  Serial.println("Ultrasonic on GPIO26(TRIG), GPIO27(ECHO)");
  Serial.println("Updated logic: 2s emergency stop, then reactivate");

  // --- Ultrasonic sensor initialization ---
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // --- Encoder initialization ---
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), onEncoder1, CHANGE);

  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), onEncoder2, CHANGE);

  pinMode(ENCODER_A3, INPUT_PULLUP);
  pinMode(ENCODER_B3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A3), onEncoder3, CHANGE);

  // --- Motor pin initialization ---
  pinMode(L298N_IN1_1, OUTPUT);
  pinMode(L298N_IN2_1, OUTPUT);
  pinMode(L298N_IN1_2, OUTPUT);
  pinMode(L298N_IN2_2, OUTPUT);
  pinMode(L298N_IN1_3, OUTPUT);
  pinMode(L298N_IN2_3, OUTPUT);

  // --- Initialize PWM ---
  ledcAttach(L298N_ENA_1, PWM_FREQ, PWM_RES);
  ledcAttach(L298N_ENA_2, PWM_FREQ, PWM_RES);
  ledcAttach(L298N_ENA_3, PWM_FREQ, PWM_RES);

  lastTime = millis();
}

void stopAllMotors() {
  for (int i = 0; i < 3; i++) {
    controlMotor(i, 0);
    // Don't reset target speeds - keep them for when we resume
    integral[i] = 0;  // Reset integral to prevent windup
  }
}

void controlMotor(int motorIndex, float output) {
  int in1_pin, in2_pin, ena_pin;
  
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

void processCommand(String cmd) {
  cmd.trim();
  
  // Check if it's direct velocity format: "vel1,vel2,vel3"
  if (cmd.indexOf(',') > 0 && cmd.indexOf(':') == -1) {
    // Parse three velocities
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    
    if (secondComma > firstComma) {
      targetSpeed_mmps[0] = cmd.substring(0, firstComma).toFloat();
      targetSpeed_mmps[1] = cmd.substring(firstComma + 1, secondComma).toFloat();
      targetSpeed_mmps[2] = cmd.substring(secondComma + 1).toFloat();
      
      Serial.print("Set velocities: ");
      Serial.print(targetSpeed_mmps[0]); Serial.print(",");
      Serial.print(targetSpeed_mmps[1]); Serial.print(",");
      Serial.println(targetSpeed_mmps[2]);
    }
  }
  // Legacy commands for PID testing
  else if (cmd.startsWith("s")) {
    if (cmd.indexOf(":") > 0) {
      int motorNum = cmd.substring(1, cmd.indexOf(":")).toInt();
      float speed = cmd.substring(cmd.indexOf(":") + 1).toFloat();
      if (motorNum >= 1 && motorNum <= 3) {
        targetSpeed_mmps[motorNum - 1] = speed;
      }
    } else {
      float speed = cmd.substring(1).toFloat();
      for (int i = 0; i < 3; i++) {
        targetSpeed_mmps[i] = speed;
      }
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

void loop() {
  // Process UART2 commands
  if (Serial2.available()) {
    String cmd = Serial2.readStringUntil('\n');
    processCommand(cmd);
  }
  
  // Process Serial commands (for debugging)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
  
  // Check ultrasonic sensor
  unsigned long now = millis();
  if (now - lastUltrasonicTime >= ULTRASONIC_PERIOD) {
    float distance = getUltrasonicDistance();
    
    // Update object detection status
    objectDetected = (distance < STOP_DISTANCE_CM);
    
    // Handle emergency stop logic
    if (objectDetected) {
      if (!emergencyStop) {
        // First time detecting object - start emergency stop
        emergencyStop = true;
        emergencyStopStartTime = now;
        stopAllMotors();
        Serial.print("EMERGENCY STOP! Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
        Serial2.println("ESTOP:ACTIVE");
      } else {
        // Already in emergency stop - check if 2 seconds have passed
        if (now - emergencyStopStartTime >= EMERGENCY_STOP_DURATION) {
          emergencyStop = false;
          Serial.println("Emergency stop timeout - motors reactivated");
          Serial2.println("ESTOP:TIMEOUT_REACTIVATED");
          // Motors will resume with existing target speeds in PID loop
        }
      }
    } else {
      // Object not detected (distance >= 20cm)
      if (emergencyStop) {
        // Clear emergency stop when object moves away
        emergencyStop = false;
        Serial.println("Object cleared - emergency stop reset");
        Serial2.println("ESTOP:CLEARED");
      }
    }
    
    lastUltrasonicTime = now;
  }
  
  // PID control loop (every 100ms)
  if (now - lastTime >= 100) {
    
    // Send velocity data format: "VEL:vel1,vel2,vel3,timestamp\n"
    Serial2.print("VEL:");
    Serial2.print(currentSpeed[0], 2); Serial2.print(",");
    Serial2.print(currentSpeed[1], 2); Serial2.print(",");
    Serial2.print(currentSpeed[2], 2); Serial2.print(",");
    Serial2.println(millis());
    
    for (int i = 0; i < 3; i++) {
      long count = encoderCount[i];
      long tickDelta = count - lastEncoderCount[i];
      currentSpeed[i] = tickDelta * 10 * mm_per_tick;
      lastEncoderCount[i] = count;

      // Only run PID if not in emergency stop
      if (!emergencyStop) {
        // PID control
        float error = targetSpeed_mmps[i] - currentSpeed[i];
        integral[i] += error * 0.1;
        float derivative = (error - previousError[i]) / 0.1;
        float output = Kp * error + Ki * integral[i] - Kd * derivative;
        previousError[i] = error;

        output = constrain(output, -PWM_MAX, PWM_MAX);
        controlMotor(i, output);
      } else {
        // Ensure motors stay stopped during emergency stop
        controlMotor(i, 0);
      }
    }

    // Display status
    Serial.print("Target1:"); Serial.print(targetSpeed_mmps[0]);
    Serial.print(",Speed1:"); Serial.print(currentSpeed[0]);
    Serial.print(",Target2:"); Serial.print(targetSpeed_mmps[1]);
    Serial.print(",Speed2:"); Serial.print(currentSpeed[1]);
    Serial.print(",Target3:"); Serial.print(targetSpeed_mmps[2]);
    Serial.print(",Speed3:"); Serial.print(currentSpeed[2]);
    if (emergencyStop) {
      unsigned long stopDuration = now - emergencyStopStartTime;
      Serial.print(" [EMERGENCY STOP - ");
      Serial.print(stopDuration / 1000.0, 1);
      Serial.print("s]");
    }
    Serial.println();

    lastTime = now;
  }
}