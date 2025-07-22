#include <Arduino.h>
#include "esp32-hal-ledc.h"

/*  
This version receives UART commands to control three motors
UART2 on GPIO16(RX), GPIO17(TX) @ 115200 baud

Command format: 
"vel1,vel2,vel3\n"  // Direct motor velocities in mm/s
Example: "100,150,200\n" sets motor1=100mm/s, motor2=150mm/s, motor3=200mm/s

Also supports legacy commands via USB Serial for testing:
"s150\n"     // Set all motors to same speed
"s1:100\n"   // Set motor 1 speed
"kp1.2\n"    // Adjust PID parameters
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

// --- PWM Settings (ESP32) ---
#define PWM_FREQ 50000
#define PWM_RES 8

// --- Encoder variables ---
volatile long encoderCount[3] = {0, 0, 0};
long lastEncoderCount[3] = {0, 0, 0};
float currentSpeed[3] = {0, 0, 0};
float totalDistance_mm[3] = {0, 0, 0};

// --- PID parameters ---
float Kp = 0.65;
float Ki = 1.0;
float Kd = 0.0;
float targetSpeed_mmps[3] = {0, 0, 0};  // Start with motors stopped
float integral[3] = {0, 0, 0};
float previousError[3] = {0, 0, 0};

// --- Limits ---
const int PWM_MAX = 204;

// --- Timing ---
unsigned long lastTime = 0;
unsigned long lastUARTReceive = 0;
unsigned long lastOdometryTime = 0;
const unsigned long UART_TIMEOUT = 1000;  // Stop motors if no command for 1 second
const unsigned long ODOMETRY_PERIOD = 50;  // Send odometry every 50ms (20Hz)

// --- Mechanical parameters ---
const float wheel_circumference_mm = 219.91;
const int ticks_per_rev = 1320;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

// --- UART buffer ---
String uartBuffer = "";

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

void setup() {
  Serial.begin(115200);  // Debug serial
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // UART2 for receiving commands
  
  Serial.println("Motor controller with UART ready");
  Serial.println("UART2 on GPIO16(RX), GPIO17(TX)");

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

void processUARTCommand(String cmd) {
  cmd.trim();
  lastUARTReceive = millis();
  
  // Check if it's direct velocity format: "vel1,vel2,vel3"
  if (cmd.indexOf(',') > 0 && cmd.indexOf(':') == -1) {
    // Parse three velocities
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    
    if (secondComma > firstComma) {
      targetSpeed_mmps[0] = cmd.substring(0, firstComma).toFloat();
      targetSpeed_mmps[1] = cmd.substring(firstComma + 1, secondComma).toFloat();
      targetSpeed_mmps[2] = cmd.substring(secondComma + 1).toFloat();
      
      Serial.print("Received velocities: ");
      Serial.print(targetSpeed_mmps[0]); Serial.print(",");
      Serial.print(targetSpeed_mmps[1]); Serial.print(",");
      Serial.println(targetSpeed_mmps[2]);
      
      // Send acknowledgment back via UART2
      Serial2.print("ACK:");
      Serial2.print(targetSpeed_mmps[0]); Serial2.print(",");
      Serial2.print(targetSpeed_mmps[1]); Serial2.print(",");
      Serial2.println(targetSpeed_mmps[2]);
    }
  }
  // Legacy commands from original code (for USB Serial testing)
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

void sendOdometry() {
  // Send velocity data format: "VEL:vel1,vel2,vel3,timestamp\n"
  Serial2.print("VEL:");
  
  // Current velocities (mm/s)
  Serial2.print(currentSpeed[0], 2); Serial2.print(",");
  Serial2.print(currentSpeed[1], 2); Serial2.print(",");
  Serial2.print(currentSpeed[2], 2); Serial2.print(",");
  
  // Timestamp in milliseconds
  Serial2.println(millis());
}

void loop() {
  // Process UART2 commands
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      processUARTCommand(uartBuffer);
      uartBuffer = "";
    } else {
      uartBuffer += c;
    }
  }
  
  // Process Serial commands (for debugging)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processUARTCommand(cmd);
  }
  
  // Safety: Stop motors if no command received for timeout period
  if (millis() - lastUARTReceive > UART_TIMEOUT && lastUARTReceive > 0) {
    for (int i = 0; i < 3; i++) {
      targetSpeed_mmps[i] = 0;
    }
  }
  
  // PID control loop (every 100ms)
  unsigned long now = millis();
  if (now - lastTime >= 100) {
    
    for (int i = 0; i < 3; i++) {
      long count = encoderCount[i];
      long tickDelta = count - lastEncoderCount[i];
      currentSpeed[i] = tickDelta * 10 * mm_per_tick;
      lastEncoderCount[i] = count;

      // PID control
      float error = targetSpeed_mmps[i] - currentSpeed[i];
      integral[i] += error * 0.1;
      float derivative = (error - previousError[i]) / 0.1;
      float output = Kp * error + Ki * integral[i] - Kd * derivative;
      previousError[i] = error;

      output = constrain(output, -PWM_MAX, PWM_MAX);
      controlMotor(i, output);

      totalDistance_mm[i] = encoderCount[i] * mm_per_tick;
    }

    // Display status
    Serial.print("Target1:"); Serial.print(targetSpeed_mmps[0]);
    Serial.print(",Speed1:"); Serial.print(currentSpeed[0]);
    Serial.print(",Target2:"); Serial.print(targetSpeed_mmps[1]);
    Serial.print(",Speed2:"); Serial.print(currentSpeed[1]);
    Serial.print(",Target3:"); Serial.print(targetSpeed_mmps[2]);
    Serial.print(",Speed3:"); Serial.println(currentSpeed[2]);

    lastTime = now;
  }
  
  // Send odometry data periodically
  if (now - lastOdometryTime >= ODOMETRY_PERIOD) {
    sendOdometry();
    lastOdometryTime = now;
  }
}