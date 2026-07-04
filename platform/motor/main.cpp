#include <Arduino.h>

// ============================================================
// ESP32 — Cytron MD20A x 4基制御 (Sign-Magnitude モード)
// ※ Arduinoコア v3.0以降の新しい ledc 構文に対応済み
//
// Serial protocol (115200 baud, '\n' terminated):
//   M<0-3>:P<0-100>  — 指定モーターの PWM デューティ設定 (例: M0:P50)
//   M<0-3>:H         — 指定モーターの DIR HIGH (例: M1:H)
//   M<0-3>:L         — 指定モーターの DIR LOW  (例: M1:L)
//   M<0-3>:STOP      — 指定モーターをソフトストップ (例: M2:STOP)
// ============================================================

static const int NUM_MOTORS = 4;

// 起動トラブルや通信干渉を起こさない安全な汎用GPIOピンに変更しました
static const int PWM_PINS[NUM_MOTORS] = {25, 26, 27, 14}; 
static const int DIR_PINS[NUM_MOTORS] = {32, 33, 12, 23}; 

static const int LEDC_FREQ = 5000;  // Hz
static const int LEDC_BITS = 8;     // 0-255

static const int SOFT_STEP_DUTY = 1;   
static const int SOFT_STEP_MS   = 50;  

// 各モーターの状態を配列で管理
static int           currentDuties[NUM_MOTORS] = {0, 0, 0, 0};
static bool          softStopping[NUM_MOTORS]  = {false, false, false, false};
static unsigned long lastStepMs[NUM_MOTORS]    = {0, 0, 0, 0};

static String serialBuf;

// ----------------------------------------------------------

static void applyDuty(int motorIdx, int duty) {
  currentDuties[motorIdx] = constrain(duty, 0, 255);
  // 新仕様: チャンネルではなくPWMの「ピン番号」を直接指定して出力します
  ledcWrite(PWM_PINS[motorIdx], currentDuties[motorIdx]);
}

static void processCmd(const String& cmd) {
  if (cmd.length() == 0) return;

  // コマンド解析 (例: "M0:P50" -> モーター番号と命令に分解)
  if (cmd[0] != 'M') return;
  
  int colonIdx = cmd.indexOf(':');
  if (colonIdx == -1) return;

  int motorIdx = cmd.substring(1, colonIdx).toInt();
  if (motorIdx < 0 || motorIdx >= NUM_MOTORS) return; // モーター番号異常は無視

  String action = cmd.substring(colonIdx + 1);

  // 各モーターに対する処理
  if (action[0] == 'P') {
    int pct  = constrain(action.substring(1).toInt(), 0, 100);
    int duty = map(pct, 0, 100, 0, 255);
    softStopping[motorIdx] = false;
    applyDuty(motorIdx, duty);
    Serial.printf("Motor %d PWM: %d%% (duty=%d)\n", motorIdx, pct, duty);

  } else if (action == "H") {
    digitalWrite(DIR_PINS[motorIdx], HIGH);
    Serial.printf("Motor %d DIR: HIGH\n", motorIdx);

  } else if (action == "L") {
    digitalWrite(DIR_PINS[motorIdx], LOW);
    Serial.printf("Motor %d DIR: LOW\n", motorIdx);

  } else if (action == "STOP") {
    if (currentDuties[motorIdx] > 0) {
      softStopping[motorIdx] = true;
      lastStepMs[motorIdx]   = millis();
      Serial.printf("Motor %d: soft stopping...\n", motorIdx);
    }
  }
}

// ----------------------------------------------------------

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);

    // 新仕様: ledcSetup/AttachPin は廃止され、ledcAttach のみになりました
    ledcAttach(PWM_PINS[i], LEDC_FREQ, LEDC_BITS);
    applyDuty(i, 0);
  }
}

void loop() {
  unsigned long now = millis();

  // 4つのモーターそれぞれのソフトストップ処理をループで回す
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (softStopping[i]) {
      if (now - lastStepMs[i] >= (unsigned long)SOFT_STEP_MS) {
        lastStepMs[i] = now;
        int next = currentDuties[i] - SOFT_STEP_DUTY;
        if (next <= 0) {
          applyDuty(i, 0);
          softStopping[i] = false;
          Serial.printf("Motor %d STOP: done\n", i);
        } else {
          applyDuty(i, next);
        }
      }
    }
  }

  // シリアル受信
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      serialBuf.trim();
      processCmd(serialBuf);
      serialBuf = "";
    } else if (c != '\r') {
      serialBuf += c;
    }
  }
}