// === 馬達控制腳位 (L298N) ===
#define MOTOR0_IN1 13
#define MOTOR0_IN2 14
#define MOTOR0_EN  15

#define MOTOR1_IN1 16
#define MOTOR1_IN2 17
#define MOTOR1_EN  18

#define MOTOR2_IN1 19
#define MOTOR2_IN2 21
#define MOTOR2_EN  22

// === 編碼器腳位 ===
#define ENCODER0_A 32
#define ENCODER1_A 33
#define ENCODER2_A 25

// === 編碼器參數 ===
const int ticks_per_rev = 1320;
const float wheel_circumference_mm = 219.91;
const float mm_per_tick = wheel_circumference_mm / ticks_per_rev;

// === 馬達狀態 ===
struct MotorState {
  volatile long ticks = 0;
  long lastTicks = 0;
  float distance_mm = 0;
  float currentSpeed = 0;
  float targetSpeed = 100; // mm/s 預設
  float integral = 0;
  float prevError = 0;
};
MotorState motors[3];

// === PID 參數 ===
const float Kp = 1.0, Ki = 0.3, Kd = 0.05;

// === 更新計時 ===
unsigned long lastUpdate = 0;
const int UPDATE_INTERVAL_MS = 100;

// === 編碼器中斷 ===
void IRAM_ATTR encoderISR0() { motors[0].ticks++; }
void IRAM_ATTR encoderISR1() { motors[1].ticks++; }
void IRAM_ATTR encoderISR2() { motors[2].ticks++; }

// === 初始化馬達腳位 ===
void setupMotorPins() {
  int motorPins[3][3] = {
    {MOTOR0_IN1, MOTOR0_IN2, MOTOR0_EN},
    {MOTOR1_IN1, MOTOR1_IN2, MOTOR1_EN},
    {MOTOR2_IN1, MOTOR2_IN2, MOTOR2_EN}
  };

  for (int i = 0; i < 3; i++) {
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
    pinMode(motorPins[i][2], OUTPUT);
  }
}

// === 馬達控制 ===
void setMotorPWM(int index, float pwm) {
  pwm = constrain(pwm, -255, 255);
  int IN1, IN2, EN;

  switch (index) {
    case 0: IN1 = MOTOR0_IN1; IN2 = MOTOR0_IN2; EN = MOTOR0_EN; break;
    case 1: IN1 = MOTOR1_IN1; IN2 = MOTOR1_IN2; EN = MOTOR1_EN; break;
    case 2: IN1 = MOTOR2_IN1; IN2 = MOTOR2_IN2; EN = MOTOR2_EN; break;
    default: return;
  }

  if (pwm >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN, pwm);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(EN, -pwm);
  }
}

// === 接收 Serial 指令 ===
void handleSerialInput() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("S")) {
      float val = cmd.substring(1).toFloat();
      for (int i = 0; i < 3; i++) motors[i].targetSpeed = val;
      Serial.printf("目標速度設為 %.1f mm/s\n", val);
    }
  }
}

// === 初始化 ===
void setup() {
  Serial.begin(115200);
  setupMotorPins();

  pinMode(ENCODER0_A, INPUT_PULLUP);
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER0_A), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoderISR2, CHANGE);
}

// === 主迴圈 ===
void loop() {
  handleSerialInput();

  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_INTERVAL_MS) {
    for (int i = 0; i < 3; i++) {
      long currentTicks = motors[i].ticks;
      long deltaTicks = currentTicks - motors[i].lastTicks;
      motors[i].lastTicks = currentTicks;

      motors[i].currentSpeed = (deltaTicks * 1000.0 / UPDATE_INTERVAL_MS) * mm_per_tick;

      float error = motors[i].targetSpeed - motors[i].currentSpeed;
      motors[i].integral += error * (UPDATE_INTERVAL_MS / 1000.0);
      float derivative = (error - motors[i].prevError) / (UPDATE_INTERVAL_MS / 1000.0);
      float output = Kp * error + Ki * motors[i].integral + Kd * derivative;
      motors[i].prevError = error;

      setMotorPWM(i, output);

      motors[i].distance_mm = motors[i].ticks * mm_per_tick;
    }

    Serial.printf("速度(mm/s): %.1f %.1f %.1f | 里程(mm): %.1f %.1f %.1f\n",
      motors[0].currentSpeed, motors[1].currentSpeed, motors[2].currentSpeed,
      motors[0].distance_mm, motors[1].distance_mm, motors[2].distance_mm);

    lastUpdate = now;
  }
}
