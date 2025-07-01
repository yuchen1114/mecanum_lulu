// --- 腳位定義 (L298N) ---
#define ENCODER_A 2
#define ENCODER_B 4
#define L298N_IN1 13
#define L298N_IN2 14
#define L298N_ENA 15  // 這是 PWM 腳位

// --- 編碼器 ---
volatile long encoderCount = 0;
long lastEncoderCount = 0;
float currentSpeed = 0;

// --- PID ---
float Kp = 1.0;
float Ki = 0.5;
float Kd = 0.1;
float targetSpeed_mmps = 100;  // 目標速度 mm/s
float integral = 0, previousError = 0;

// --- 時間 ---
unsigned long lastTime = 0;

// --- 機構參數 ---
const float wheel_circumference_mm = 219.91;
const int ticks_per_rev = 1320;

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

  // 馬達控制腳
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_ENA, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), onEncoder, CHANGE);
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= 100) {
    // 1. 讀取速度
    long count = encoderCount;
    long tickSpeed = (count - lastEncoderCount) * 10; // ticks/sec
    float mm_per_tick = wheel_circumference_mm / ticks_per_rev;
    currentSpeed = tickSpeed * mm_per_tick;
    lastEncoderCount = count;

    // 2. PID 控制
    float error = targetSpeed_mmps - currentSpeed;
    integral += error * 0.1;
    float derivative = (error - previousError) / 0.1;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;

    output = constrain(output, -255, 255);

    // 3. 控制 L298N 馬達
    if (output >= 0) {
      digitalWrite(L298N_IN1, HIGH);
      digitalWrite(L298N_IN2, LOW);
      analogWrite(L298N_ENA, output);
    } else {
      digitalWrite(L298N_IN1, LOW);
      digitalWrite(L298N_IN2, HIGH);
      analogWrite(L298N_ENA, -output);
    }

    // 4. 顯示速度
    Serial.print("目標(mm/s):");
    Serial.print(targetSpeed_mmps);
    Serial.print(", 實際(mm/s):");
    Serial.println(currentSpeed);

    lastTime = now;
  }
}
