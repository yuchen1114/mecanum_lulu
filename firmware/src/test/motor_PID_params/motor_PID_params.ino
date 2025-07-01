/*  

Arduino IDE → 工具 → Serial Plotter

command window
s150     // 設定目標速度為 150 mm/s
kp1.2    // 設定比例增益
ki0.3    // 設定積分增益
kd0.05   // 設定微分增益

*/

// --- 腳位定義 (L298N) ---
#define ENCODER_A 2
#define ENCODER_B 4
#define L298N_IN1 13
#define L298N_IN2 14
#define L298N_ENA 15  // PWM 腳位

// --- 編碼器變數 ---
volatile long encoderCount = 0;
long lastEncoderCount = 0;
float currentSpeed = 0;
float totalDistance_mm = 0;  // 累積距離

// --- PID ---
float Kp = 1.0;
float Ki = 0.5;
float Kd = 0.1;
float targetSpeed_mmps = 100;
float integral = 0, previousError = 0;

// --- 限制參數 ---
const int PWM_MAX = 204; // 約 80% 的 255

// --- 時間 ---
unsigned long lastTime = 0;

// --- 機構參數 ---
const float wheel_circumference_mm = 219.91;
const int ticks_per_rev = 1320;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

void IRAM_ATTR onEncoder() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  if (A == B) encoderCount++;
  else encoderCount--;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_ENA, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), onEncoder, CHANGE);

  lastTime = millis();
  Serial.println("Ready. 輸入 s100/kp1.2/ki0.3/kd0.05 調整參數");
}

void loop() {
  // === 處理 Serial 輸入參數 ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("s")) {
      targetSpeed_mmps = cmd.substring(1).toFloat();
      Serial.print("設定目標速度為：");
      Serial.println(targetSpeed_mmps);
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
    long count = encoderCount;
    long tickDelta = count - lastEncoderCount;
    currentSpeed = tickDelta * 10 * mm_per_tick;
    lastEncoderCount = count;

    // PID 控制
    float error = targetSpeed_mmps - currentSpeed;
    integral += error * 0.1;
    float derivative = (error - previousError) / 0.1;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    output = constrain(output, -PWM_MAX, PWM_MAX); // 限制 80% duty

    // 馬達控制
    if (output >= 0) {
      digitalWrite(L298N_IN1, HIGH);
      digitalWrite(L298N_IN2, LOW);
      analogWrite(L298N_ENA, output);
    } else {
      digitalWrite(L298N_IN1, LOW);
      digitalWrite(L298N_IN2, HIGH);
      analogWrite(L298N_ENA, -output);
    }

    // 累積距離更新
    totalDistance_mm = encoderCount * mm_per_tick;

    // 顯示：Serial Plotter 可視化速度 + 監看里程
    Serial.print(targetSpeed_mmps);   // Plotter Y1
    Serial.print(",");
    Serial.print(currentSpeed);       // Plotter Y2
    Serial.print(",");
    Serial.println(totalDistance_mm); // 可於序列監控器觀看

    lastTime = now;
  }
}
