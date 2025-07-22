#include <Arduino.h>
#include "esp32-hal-ledc.h"  // Add this line for ESP32 LEDC functions

/*  
Arduino IDE → 工具 → Serial Plotter

指令範例：
s150     // 設定所有馬達目標速度為 150 mm/s
s1:100   // 設定馬達1目標速度為 100 mm/s
s2:200   // 設定馬達2目標速度為 200 mm/s
s3:150   // 設定馬達3目標速度為 150 mm/s
kp1.2    // 設定所有馬達比例增益
ki0.3    // 設定所有馬達積分增益
kd0.05   // 設定所有馬達微分增益
*/

// // --- 腳位定義 (三個 L298N 模組) ---
// // 馬達1 19~16
// #define ENCODER_A1 19   //15
// #define ENCODER_B1 18
// #define L298N_ENA_1 5   //13
// #define L298N_IN1_1 8   //old17
// #define L298N_IN2_1 7   //old16

// // 馬達2 34~25
// #define ENCODER_B2 34  //18
// #define ENCODER_A2 35
// #define L298N_ENA_2 32  //16
// #define L298N_IN2_2 33
// #define L298N_IN1_2 25

// // 馬達3 26~13
// #define ENCODER_B3 26  //13
// #define ENCODER_A3 27
// #define L298N_IN2_3 14  //11
// #define L298N_IN1_3 12  //10
// #define L298N_ENA_3 13  //8
// --- 腳位定義 (三個 L298N 模組) ---
// 馬達1 - 使用有內部上拉的腳位
#define ENCODER_A1 15   // From ultrasonic TRIG_PIN_1 - has pullup
#define ENCODER_B1 2    // From ultrasonic ECHO_PIN_1 - has pullup
#define L298N_ENA_1 5   // OK (PWM)
#define L298N_IN1_1 19  // Safe pin
#define L298N_IN2_1 18  // Safe pin

// 馬達2 - 保持使用 input-only pins (這些已經正常工作)
#define ENCODER_A2 35   // OK (input only)
#define ENCODER_B2 34   // OK (input only)
#define L298N_ENA_2 32  // OK (PWM)
#define L298N_IN1_2 25  // OK
#define L298N_IN2_2 33  // OK

// 馬達3 - 使用有內部上拉的腳位
#define ENCODER_A3 22   // From ultrasonic TRIG_PIN_3 - has pullup
#define ENCODER_B3 23   // From ultrasonic ECHO_PIN_3 - has pullup
#define L298N_ENA_3 13  // OK (PWM)
#define L298N_IN1_3 21  // Safe pin
#define L298N_IN2_3 14  // OK


// --- PWM 設定 (ESP32 專用) ---
#define PWM_FREQ 50000
#define PWM_RES 8  // 8-bit resolution: 0–255

// --- 編碼器變數 (三個馬達) ---
volatile long encoderCount[3] = {0, 0, 0};
long lastEncoderCount[3] = {0, 0, 0};
float currentSpeed[3] = {0, 0, 0};
float totalDistance_mm[3] = {0, 0, 0};

// --- PID 參數 (三個馬達) ---
float Kp = 0.65;
float Ki = 1.0;
float Kd = 0.0;
float targetSpeed_mmps[3] = {100, 100, 100};
float integral[3] = {0, 0, 0};
float previousError[3] = {0, 0, 0};

// --- 限制參數 ---
const int PWM_MAX = 204; // 約 80% 的 255

// --- 時間 ---
unsigned long lastTime = 0;

// --- 機構參數 ---
const float wheel_circumference_mm = 219.91;
const int ticks_per_rev = 1320;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

// --- 中斷服務函數 ---
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

  // // --- 編碼器初始化 ---
  // pinMode(ENCODER_A1, INPUT_PULLUP);
  // pinMode(ENCODER_B1, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_A1), onEncoder1, CHANGE);

  // pinMode(ENCODER_A2, INPUT_PULLUP);
  // pinMode(ENCODER_B2, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_A2), onEncoder2, CHANGE);

  // pinMode(ENCODER_A3, INPUT_PULLUP);
  // pinMode(ENCODER_B3, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_A3), onEncoder3, CHANGE);
  // --- 編碼器初始化 ---
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), onEncoder1, CHANGE);

  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), onEncoder2, CHANGE);

  pinMode(ENCODER_A3, INPUT_PULLUP);
  pinMode(ENCODER_B3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A3), onEncoder3, CHANGE);

  // --- 馬達腳位初始化 ---
  pinMode(L298N_IN1_1, OUTPUT);
  pinMode(L298N_IN2_1, OUTPUT);
  pinMode(L298N_IN1_2, OUTPUT);
  pinMode(L298N_IN2_2, OUTPUT);
  pinMode(L298N_IN1_3, OUTPUT);
  pinMode(L298N_IN2_3, OUTPUT);

  // --- 初始化 PWM (ESP32 Core 3.0+ syntax) ---
  ledcAttach(L298N_ENA_1, PWM_FREQ, PWM_RES);  // 馬達1 PWM
  ledcAttach(L298N_ENA_2, PWM_FREQ, PWM_RES);  // 馬達2 PWM
  ledcAttach(L298N_ENA_3, PWM_FREQ, PWM_RES);  // 馬達3 PWM

  lastTime = millis();
  Serial.println("Ready. 輸入指令:");
  Serial.println("s150 (所有馬達速度), s1:100 (馬達1速度), s2:200 (馬達2速度), s3:150 (馬達3速度)");
  Serial.println("kp1.2/ki0.3/kd0.05 調整 PID 參數");
}

void controlMotor(int motorIndex, float output) {
  int in1_pin, in2_pin, ena_pin;
  
  // 選擇對應的腳位
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
  
  // 馬達控制
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

void loop() {
  // === 處理 Serial 輸入參數 ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("s")) {
      if (cmd.indexOf(":") > 0) {
        // 單獨設定某個馬達: s1:100, s2:200, s3:150
        int motorNum = cmd.substring(1, cmd.indexOf(":")).toInt();
        float speed = cmd.substring(cmd.indexOf(":") + 1).toFloat();
        if (motorNum >= 1 && motorNum <= 3) {
          targetSpeed_mmps[motorNum - 1] = speed;
          Serial.print("設定馬達"); Serial.print(motorNum); 
          Serial.print("目標速度為："); Serial.println(speed);
        }
      } else {
        // 設定所有馬達相同速度: s150
        float speed = cmd.substring(1).toFloat();
        for (int i = 0; i < 3; i++) {
          targetSpeed_mmps[i] = speed;
        }
        Serial.print("設定所有馬達目標速度為："); Serial.println(speed);
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

  // === 每 100ms 更新一次 PID 控制與里程 ===
  unsigned long now = millis();
  if (now - lastTime >= 100) {
    
    // 處理三個馬達的 PID 控制
    for (int i = 0; i < 3; i++) {
      long count = encoderCount[i];
      long tickDelta = count - lastEncoderCount[i];
      currentSpeed[i] = tickDelta * 10 * mm_per_tick;
      lastEncoderCount[i] = count;

      // PID 控制
      float error = targetSpeed_mmps[i] - currentSpeed[i];
      integral[i] += error * 0.1;
      float derivative = (error - previousError[i]) / 0.1;
      float output = Kp * error + Ki * integral[i] - Kd * derivative;
      previousError[i] = error;

      output = constrain(output, -PWM_MAX, PWM_MAX); // 限制輸出

      // 控制馬達
      controlMotor(i, output);

      // 累積距離更新
      totalDistance_mm[i] = encoderCount[i] * mm_per_tick;
    }

    // 顯示到 Serial Plotter (目標速度和實際速度)
    Serial.print("Target1:"); Serial.print(targetSpeed_mmps[0]);
    Serial.print(",Speed1:"); Serial.print(currentSpeed[0]);
    Serial.print(",Target2:"); Serial.print(targetSpeed_mmps[1]);
    Serial.print(",Speed2:"); Serial.print(currentSpeed[1]);
    Serial.print(",Target3:"); Serial.print(targetSpeed_mmps[2]);
    Serial.print(",Speed3:"); Serial.println(currentSpeed[2]);

    lastTime = now;
  }
}