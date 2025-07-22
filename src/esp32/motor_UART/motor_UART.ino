/*
接收來自UART的馬達速度指令並控制三顆馬達
ESP32 UART2 使用 GPIO16(RX), GPIO17(TX)
訊息格式: "speed1,speed2,speed3" (例如: "150.5,-200.0,100.0") 單位: mm/s
同時提供網頁監控介面
*/

//IP fixed as 192.168.16.184

#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include "esp32-hal-ledc.h"  // ESP32 LEDC functions

// WiFi設定
const char* ssid = "Wifi662-8_2B";
const char* password = "0980566079";
IPAddress local_IP(192, 168, 16, 184);   // 固定 IP
IPAddress gateway(192, 168, 16, 1);      // 路由器 IP
IPAddress subnet(255, 255, 255, 0);      // 子網掩碼

WebServer server(80);
String lastMsg = "No message received yet";
unsigned long lastMsgTime = 0;

// 接收的馬達速度變數
float receivedSpeed[3] = {0.0, 0.0, 0.0};

// --- 腳位定義 (三個 L298N 模組) ---
// 馬達1
#define ENCODER_A1 19
#define ENCODER_B1 18
#define L298N_ENA_1 5
#define L298N_IN1_1 17
#define L298N_IN2_1 16

// 馬達2
#define ENCODER_B2 34
#define ENCODER_A2 35
#define L298N_ENA_2 32
#define L298N_IN2_2 33
#define L298N_IN1_2 25

// 馬達3
#define ENCODER_B3 26
#define ENCODER_A3 27
#define L298N_IN2_3 14
#define L298N_IN1_3 12
#define L298N_ENA_3 13

// --- PWM 設定 ---
#define PWM_FREQ 50000
#define PWM_RES 8  // 8-bit resolution: 0–255

// --- 編碼器變數 ---
volatile long encoderCount[3] = {0, 0, 0};
long lastEncoderCount[3] = {0, 0, 0};
float currentSpeed[3] = {0, 0, 0};

// --- PID 參數 ---
float Kp = 0.65;
float Ki = 1.0;
float Kd = 0.0;
float targetSpeed_mmps[3] = {0, 0, 0};  // 初始目標速度為0
float integral[3] = {0, 0, 0};
float previousError[3] = {0, 0, 0};

// --- 限制參數 ---
const int PWM_MAX = 204; // 約 80% 的 255
const float MAX_SPEED = 300.0; // 最大速度限制 mm/s

// --- 時間 ---
unsigned long lastTime = 0;
unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT = 1000; // 1秒無指令則停止

// --- 機構參數 ---
const float wheel_circumference_mm = 219.91;
const int ticks_per_rev = 1320;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

// 速度限制參數
const float SPEED_SCALE = 1.0; // 如果需要縮放接收到的速度值

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

// 網頁處理函數
void handleRoot() {
  String html = "<html><head><meta http-equiv='refresh' content='2'></head><body>";
  html += "<h1>ESP32 Direct Motor Speed Controller</h1>";
  html += "<h2>UART Status</h2>";
  html += "<p><strong>Last received:</strong> " + lastMsg + "</p>";
  html += "<p><strong>Time:</strong> " + String(millis()) + " ms</p>";
  html += "<h2>Received Wheel Speeds (mm/s)</h2>";
  html += "<p><strong>Motor 1 Received:</strong> " + String(receivedSpeed[0], 1) + "</p>";
  html += "<p><strong>Motor 2 Received:</strong> " + String(receivedSpeed[1], 1) + "</p>";
  html += "<p><strong>Motor 3 Received:</strong> " + String(receivedSpeed[2], 1) + "</p>";
  html += "<h2>Motor Speeds (mm/s)</h2>";
  html += "<p><strong>Motor 1 Target:</strong> " + String(targetSpeed_mmps[0], 1) + "</p>";
  html += "<p><strong>Motor 1 Current:</strong> " + String(currentSpeed[0], 1) + "</p>";
  html += "<p><strong>Motor 2 Target:</strong> " + String(targetSpeed_mmps[1], 1) + "</p>";
  html += "<p><strong>Motor 2 Current:</strong> " + String(currentSpeed[1], 1) + "</p>";
  html += "<p><strong>Motor 3 Target:</strong> " + String(targetSpeed_mmps[2], 1) + "</p>";
  html += "<p><strong>Motor 3 Current:</strong> " + String(currentSpeed[2], 1) + "</p>";
  html += "<p><strong>UART Config:</strong> GPIO16(RX), GPIO17(TX) @ 115200 baud</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
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
    ledcWrite(ena_pin, (int)output);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    ledcWrite(ena_pin, (int)(-output));
  }
}

// 直接設定馬達目標速度
void setMotorSpeeds(float speed1, float speed2, float speed3) {
  // 應用縮放係數並限制速度
  targetSpeed_mmps[0] = constrain(speed1 * SPEED_SCALE, -MAX_SPEED, MAX_SPEED);
  targetSpeed_mmps[1] = constrain(speed2 * SPEED_SCALE, -MAX_SPEED, MAX_SPEED);
  targetSpeed_mmps[2] = constrain(speed3 * SPEED_SCALE, -MAX_SPEED, MAX_SPEED);
  
  // 記錄接收到的速度值
  receivedSpeed[0] = speed1;
  receivedSpeed[1] = speed2;
  receivedSpeed[2] = speed3;
}

// 解析UART訊息 - 直接接收三個馬達速度
void parseUARTMessage(String msg) {
  // 訊息格式: "speed1,speed2,speed3" (例如: "150.5,-200.0,100.0")
  int firstComma = msg.indexOf(',');
  int secondComma = msg.indexOf(',', firstComma + 1);
  
  if (firstComma != -1 && secondComma != -1) {
    float speed1 = msg.substring(0, firstComma).toFloat();
    float speed2 = msg.substring(firstComma + 1, secondComma).toFloat();
    float speed3 = msg.substring(secondComma + 1).toFloat();
    
    // 直接設定馬達速度
    setMotorSpeeds(speed1, speed2, speed3);
    
    lastCmdTime = millis();
    
    Serial.println("Parsed wheel speeds - M1:" + String(speed1, 1) + 
                  " M2:" + String(speed2, 1) + " M3:" + String(speed3, 1));
    Serial.println("Applied targets - M1:" + String(targetSpeed_mmps[0], 1) + 
                  " M2:" + String(targetSpeed_mmps[1], 1) + " M3:" + String(targetSpeed_mmps[2], 1));
  } else {
    Serial.println("Invalid message format. Expected: speed1,speed2,speed3");
  }
}

void setup() {
  Serial.begin(115200);  // Debug serial
  delay(1000);
  Serial.println("ESP32 Direct Motor Speed Controller starting...");
  
  // 初始化 UART2，使用 GPIO16(RX), GPIO17(TX)
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("UART2 initialized on GPIO16(RX), GPIO17(TX)");

  // --- 編碼器初始化 ---
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), onEncoder1, CHANGE);

  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);
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

  // --- 初始化 PWM ---
  ledcAttach(L298N_ENA_1, PWM_FREQ, PWM_RES);
  ledcAttach(L298N_ENA_2, PWM_FREQ, PWM_RES);
  ledcAttach(L298N_ENA_3, PWM_FREQ, PWM_RES);

  // 設定靜態 IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // 啟動網頁伺服器
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started");
  
  lastTime = millis();
  lastCmdTime = millis();
  Serial.println("Ready to receive wheel speed commands via UART");
  Serial.println("Message format: speed1,speed2,speed3 (e.g., 150.5,-200.0,100.0)");
}

void loop() {
  server.handleClient();

  // 讀取 UART2 訊息
  if (Serial2.available()) {
    String receivedMsg = Serial2.readStringUntil('\n');
    receivedMsg.trim();
    lastMsg = receivedMsg;
    lastMsgTime = millis();
    
    Serial.println("Received via UART2: " + receivedMsg);
    
    // 解析並應用到馬達控制
    parseUARTMessage(receivedMsg);
    
    // 發送確認回應
    Serial2.println("ACK: " + receivedMsg);
  }
  
  // 檢查指令超時 - 如果超過1秒沒收到指令則停止馬達
  if (millis() - lastCmdTime > CMD_TIMEOUT) {
    for (int i = 0; i < 3; i++) {
      targetSpeed_mmps[i] = 0;
    }
  }
  
  // 每 100ms 更新一次 PID 控制
  unsigned long now = millis();
  if (now - lastTime >= 100) {
    
    // 處理三個馬達的 PID 控制
    for (int i = 0; i < 3; i++) {
      long count = encoderCount[i];
      long tickDelta = count - lastEncoderCount[i];
      currentSpeed[i] = tickDelta * 10 * mm_per_tick;  // 10Hz更新頻率
      lastEncoderCount[i] = count;

      // PID 控制
      float error = targetSpeed_mmps[i] - currentSpeed[i];
      integral[i] += error * 0.1;  // dt = 0.1s
      
      // 防止積分飽和
      integral[i] = constrain(integral[i], -100, 100);
      
      float derivative = (error - previousError[i]) / 0.1;
      float output = Kp * error + Ki * integral[i] + Kd * derivative;
      previousError[i] = error;

      output = constrain(output, -PWM_MAX, PWM_MAX);

      // 控制馬達
      controlMotor(i, output);
    }

    lastTime = now;
  }
  
  // 清除過時的UART訊息 (5秒後顯示 "No recent message")
  if (millis() - lastMsgTime > 5000 && lastMsg != "No message received yet") {
    lastMsg = "No recent message (timeout)";
  }
}