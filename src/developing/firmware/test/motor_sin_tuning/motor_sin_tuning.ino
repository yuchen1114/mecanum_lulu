#include <Arduino.h>
#include "esp32-hal-ledc.h"
//kp:2.0 ki:1.5 kd:0.0
/*  
PID Tuning Program with Sinusoidal Target Velocity
Optimized for Arduino Serial Plotter visualization

This program generates sinusoidal target velocities for PID tuning.
Commands via Serial Monitor:
- "sin_amp:100" - Set sine wave amplitude (mm/s)
- "sin_freq:0.5" - Set sine wave frequency (Hz)
- "sin_offset:50" - Set sine wave offset (mm/s)
- "kp:1.2" - Set Kp parameter
- "ki:0.8" - Set Ki parameter  
- "kd:0.1" - Set Kd parameter
- "motor:1" - Select motor to tune (1, 2, or 3)
- "start" - Start sine wave generation
- "stop" - Stop and set target to 0

Serial Plotter output format:
Target:xxx,Actual:xxx,Error:xxx,Output:xxx,Kp:xxx,Ki:xxx,Kd:xxx
*/

// --- Pin definitions (from your original code) ---
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

// --- PWM Settings ---
#define PWM_FREQ 50000
#define PWM_RES 8
const int PWM_MAX = 204;

// --- Encoder variables ---
volatile long encoderCount[3] = {0, 0, 0};
long lastEncoderCount[3] = {0, 0, 0};
float currentSpeed[3] = {0, 0, 0};

// --- PID parameters ---
float Kp = 1.0;
float Ki = 0.5;
float Kd = 0.0;
float integral[3] = {0, 0, 0};
float previousError[3] = {0, 0, 0};

// --- Sinusoidal target parameters ---
float sin_amplitude = 100.0;  // mm/s
float sin_frequency = 0.2;    // Hz (0.2 Hz = 5 second period)
float sin_offset = 0.0;       // mm/s
bool sine_active = false;
unsigned long sine_start_time = 0;

// --- Tuning settings ---
int active_motor = 0;  // Motor being tuned (0=motor1, 1=motor2, 2=motor3)

// --- Timing ---
unsigned long lastTime = 0;
const unsigned long PID_PERIOD = 50;  // 50ms = 20Hz update rate

// --- Mechanical parameters ---
const float wheel_circumference_mm = 219.91;
const int ticks_per_rev = 1320;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

// --- Current control values for plotting ---
float currentTarget = 0.0;
float currentError = 0.0;
float currentOutput = 0.0;

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
  Serial.begin(115200);
  
  Serial.println("=== PID Tuning with Sinusoidal Target ===");
  Serial.println("Commands:");
  Serial.println("  sin_amp:100    - Set amplitude (mm/s)");
  Serial.println("  sin_freq:0.2   - Set frequency (Hz)");
  Serial.println("  sin_offset:0   - Set offset (mm/s)");
  Serial.println("  kp:1.0         - Set Kp");
  Serial.println("  ki:0.5         - Set Ki");
  Serial.println("  kd:0.0         - Set Kd");
  Serial.println("  motor:1        - Select motor (1,2,3)");
  Serial.println("  start          - Start sine wave");
  Serial.println("  stop           - Stop sine wave");
  Serial.println();
  Serial.println("Open Serial Plotter for real-time visualization!");
  Serial.println("Current settings:");
  printCurrentSettings();

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

  // Stop all motors initially
  stopAllMotors();
  
  lastTime = millis();
}

void printCurrentSettings() {
  Serial.println("--- Current Settings ---");
  Serial.print("Active Motor: "); Serial.println(active_motor + 1);
  Serial.print("Sine Amplitude: "); Serial.print(sin_amplitude); Serial.println(" mm/s");
  Serial.print("Sine Frequency: "); Serial.print(sin_frequency); Serial.println(" Hz");
  Serial.print("Sine Offset: "); Serial.print(sin_offset); Serial.println(" mm/s");
  Serial.print("Kp: "); Serial.println(Kp, 3);
  Serial.print("Ki: "); Serial.println(Ki, 3);
  Serial.print("Kd: "); Serial.println(Kd, 3);
  Serial.print("Sine Active: "); Serial.println(sine_active ? "YES" : "NO");
  Serial.println("------------------------");
}

void stopAllMotors() {
  for (int i = 0; i < 3; i++) {
    controlMotor(i, 0);
    integral[i] = 0;
    previousError[i] = 0;
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
  
  // Constrain output
  output = constrain(output, -PWM_MAX, PWM_MAX);
  
  if (output >= 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    ledcWrite(ena_pin, (int)output);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    ledcWrite(ena_pin, (int)(-output));
  }
}

float calculateSineTarget() {
  if (!sine_active) return 0.0;
  
  unsigned long elapsed = millis() - sine_start_time;
  float time_seconds = elapsed / 1000.0;
  
  return sin_offset + sin_amplitude * sin(2.0 * PI * sin_frequency * time_seconds);
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  
  if (cmd.startsWith("sin_amp:")) {
    sin_amplitude = cmd.substring(8).toFloat();
    Serial.print("Sine amplitude set to: "); Serial.println(sin_amplitude);
  }
  else if (cmd.startsWith("sin_freq:")) {
    sin_frequency = cmd.substring(9).toFloat();
    Serial.print("Sine frequency set to: "); Serial.println(sin_frequency);
  }
  else if (cmd.startsWith("sin_offset:")) {
    sin_offset = cmd.substring(11).toFloat();
    Serial.print("Sine offset set to: "); Serial.println(sin_offset);
  }
  else if (cmd.startsWith("kp:")) {
    Kp = cmd.substring(3).toFloat();
    Serial.print("Kp set to: "); Serial.println(Kp, 3);
    // Reset integral to prevent windup when changing parameters
    integral[active_motor] = 0;
  }
  else if (cmd.startsWith("ki:")) {
    Ki = cmd.substring(3).toFloat();
    Serial.print("Ki set to: "); Serial.println(Ki, 3);
    integral[active_motor] = 0;
  }
  else if (cmd.startsWith("kd:")) {
    Kd = cmd.substring(3).toFloat();
    Serial.print("Kd set to: "); Serial.println(Kd, 3);
    previousError[active_motor] = 0;
  }
  else if (cmd.startsWith("motor:")) {
    int motor_num = cmd.substring(6).toInt();
    if (motor_num >= 1 && motor_num <= 3) {
      active_motor = motor_num - 1;
      Serial.print("Active motor set to: "); Serial.println(motor_num);
      // Reset PID for new motor
      integral[active_motor] = 0;
      previousError[active_motor] = 0;
    }
  }
  else if (cmd == "start") {
    sine_active = true;
    sine_start_time = millis();
    Serial.println("Sine wave generation STARTED");
  }
  else if (cmd == "stop") {
    sine_active = false;
    stopAllMotors();
    Serial.println("Sine wave generation STOPPED");
  }
  else if (cmd == "status" || cmd == "settings") {
    printCurrentSettings();
  }
  else if (cmd == "help") {
    Serial.println("Available commands:");
    Serial.println("  sin_amp:VALUE, sin_freq:VALUE, sin_offset:VALUE");
    Serial.println("  kp:VALUE, ki:VALUE, kd:VALUE");
    Serial.println("  motor:1-3, start, stop, status");
  }
  else {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}

void loop() {
  // Process serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
  
  unsigned long now = millis();
  
  // PID control loop
  if (now - lastTime >= PID_PERIOD) {
    // Calculate current speed for active motor
    long count = encoderCount[active_motor];
    long tickDelta = count - lastEncoderCount[active_motor];
    float dt = (now - lastTime) / 1000.0;  // Convert to seconds
    currentSpeed[active_motor] = tickDelta * mm_per_tick / dt;
    lastEncoderCount[active_motor] = count;
    
    // Calculate sine wave target
    currentTarget = calculateSineTarget();
    
    // PID calculation for active motor
    currentError = currentTarget - currentSpeed[active_motor];
    integral[active_motor] += currentError * dt;
    
    // Prevent integral windup
    float maxIntegral = PWM_MAX / (Ki + 0.001);  // Avoid division by zero
    integral[active_motor] = constrain(integral[active_motor], -maxIntegral, maxIntegral);
    
    float derivative = (currentError - previousError[active_motor]) / dt;
    currentOutput = Kp * currentError + Ki * integral[active_motor] + Kd * derivative;
    previousError[active_motor] = currentError;
    
    // Apply control signal to active motor only
    for (int i = 0; i < 3; i++) {
      if (i == active_motor) {
        controlMotor(i, currentOutput);
      } else {
        controlMotor(i, 0);  // Keep other motors stopped
      }
    }
    
    // Output data in format optimized for Serial Plotter
    // Format: Target:xxx,Actual:xxx,Error:xxx,Output:xxx,Kp:xxx,Ki:xxx,Kd:xxx
    Serial.print("Target:");
    Serial.print(currentTarget, 2);
    Serial.print(",Actual:");
    Serial.print(currentSpeed[active_motor], 2);
    Serial.print(",Error:");
    Serial.print(currentError, 2);
    Serial.print(",Output:");
    Serial.print(currentOutput, 2);
    Serial.print(",Kp:");
    Serial.print(Kp * 100, 1);  // Scale for better visualization
    Serial.print(",Ki:");
    Serial.print(Ki * 100, 1);  // Scale for better visualization
    Serial.print(",Kd:");
    Serial.println(Kd * 100, 1);  // Scale for better visualization
    
    lastTime = now;
  }
}