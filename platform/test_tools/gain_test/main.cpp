#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <driver/twai.h>

// ============================================================
// ESP32 + IM920sL + RoboMaster C620 x4 + M3508 P19 x4
//
// Raspberry Pi + PS5 Controller
//   ↓ IM920-HAT / 920MHz
// ESP32 + IM920sL
//   ↓ CAN 1Mbps (TJA1051)
// C620 x4 + M3508 x4
//
// 春ロボコンと同じ配置:
//   FL 左前  -> C620 ID1
//   FR 右前  -> C620 ID3
//   RL 左後  -> C620 ID2
//   RR 右後  -> C620 ID4
//
// 受信パケット:
//   BUTTON: 42 BUTTON_ID STATE
//   DPAD  : 44 AXIS VALUE
//   JOY   : 4A LX LY RX RY L2 R2 DPAD_X DPAD_Y
//
// このテストコードではスティック入力を足回りに使用しません。
// 十字キーだけで一定rpmを出し、再現性のある調整を行います。
// ============================================================

HardwareSerial IM920(2);

// ------------------------------------------------------------
// IM920 / LED
// ------------------------------------------------------------
static const int IM920_RX = 16;
static const int IM920_TX = 17;

static const int LED_PIN = 2;
static const uint32_t LED_PULSE_MS = 30;

// テスト中はサーボを動かさない。
// PCA9685を初期化し、CH0～CH15をFULL OFFにする。
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;
static const uint8_t PCA9685_ADDRESS = 0x40;
Adafruit_PWMServoDriver servoDriver(PCA9685_ADDRESS);

void disableAllServoOutputs() {
  for (uint8_t channel = 0; channel < 16; channel++) {
    servoDriver.setPWM(channel, 0, 4096);
  }
}

// ------------------------------------------------------------
// CAN / C620
// ------------------------------------------------------------
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_4;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_5;

static const uint32_t C620_COMMAND_ID = 0x200;
static const uint32_t C620_FEEDBACK_ID_BASE = 0x201;

static const int NUM_MOTORS = 4;
static const int NUM_WHEELS = 4;

// wheel index: FL, FR, RL, RR
// motor index: C620 ID1, ID2, ID3, ID4
static const uint8_t WHEEL_TO_MOTOR[NUM_WHEELS] = {0, 2, 1, 3};
static const uint8_t WHEEL_ESC_ID[NUM_WHEELS] = {1, 3, 2, 4};
static const char* WHEEL_NAME[NUM_WHEELS] = {"FL", "FR", "RL", "RR"};

// C620 ID順。春ロボコンと同じ反転設定。
static const bool MOTOR_REVERSED[NUM_MOTORS] = {
  false,  // ID1
  false,  // ID2
  true,   // ID3
  true    // ID4
};

// 物理的な車輪方向の符号。
static const int8_t STRAFE_SIGN[NUM_WHEELS] = {+1, -1, -1, +1};

// ------------------------------------------------------------
// 制御周期・安全設定
// ------------------------------------------------------------
static const uint32_t MOTOR_CONTROL_INTERVAL_US = 5000;  // 200Hz
static const uint32_t CAN_TX_INTERVAL_US = 2000;         // 500Hz

static const uint32_t RADIO_TIMEOUT_MS = 600;
static const uint32_t FEEDBACK_TIMEOUT_MS = 100;
static const uint8_t MOTOR_TEMP_STOP_C = 80;

static const float PID_INTEGRAL_LIMIT = 8000.0f;
static const float TARGET_RPM_SLEW_PER_SEC = 4000.0f;

static const int16_t HARD_CURRENT_LIMIT = 7000;
static const int16_t HARD_TARGET_RPM_LIMIT = 8000;

// ------------------------------------------------------------
// 返信周期
// ------------------------------------------------------------
static const bool ENABLE_REPLY_TO_PI = true;
static const uint32_t TELEMETRY_INTERVAL_MS = 600;

// ------------------------------------------------------------
// 共通状態
// ------------------------------------------------------------
String imLine = "";

bool ledOn = false;
uint32_t ledOffAt = 0;

bool canReady = false;
bool motorsActive = false;
bool estopLatched = false;

uint32_t lastRadioRxMs = 0;
uint32_t lastMotorControlUs = 0;
uint32_t lastCanTxUs = 0;
uint32_t lastCanErrorPrintMs = 0;
uint32_t lastTelemetryMs = 0;
uint8_t telemetryWheelIndex = 0;

int8_t lastDpadX = 0;
int8_t lastDpadY = 0;

// C620 ID順
float requestedMotorRpm[NUM_MOTORS] = {0, 0, 0, 0};
float rampedMotorRpm[NUM_MOTORS] = {0, 0, 0, 0};
float pidIntegral[NUM_MOTORS] = {0, 0, 0, 0};
int16_t currentCommands[NUM_MOTORS] = {0, 0, 0, 0};

uint16_t motorRotorAngle[NUM_MOTORS] = {0, 0, 0, 0};
int16_t motorRawRpm[NUM_MOTORS] = {0, 0, 0, 0};
int16_t motorMeasuredCurrent[NUM_MOTORS] = {0, 0, 0, 0};
uint8_t motorTemperature[NUM_MOTORS] = {0, 0, 0, 0};
bool motorFeedbackValid[NUM_MOTORS] = {false, false, false, false};
uint32_t motorFeedbackMs[NUM_MOTORS] = {0, 0, 0, 0};

// ============================================================
// Utility
// ============================================================

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

int16_t clampInt16(int32_t value, int16_t minimum, int16_t maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return (int16_t)value;
}

float applyMotorInverse(int motorIndex, float value) {
  return MOTOR_REVERSED[motorIndex] ? -value : value;
}

float moveToward(float current, float target, float maxChange) {
  float difference = target - current;

  if (difference > maxChange) return current + maxChange;
  if (difference < -maxChange) return current - maxChange;
  return target;
}

int16_t readInt16BigEndian(uint8_t highByte, uint8_t lowByte) {
  return (int16_t)(((uint16_t)highByte << 8) | lowByte);
}

bool feedbackFresh(int motorIndex) {
  return motorFeedbackValid[motorIndex] &&
         (millis() - motorFeedbackMs[motorIndex] <= FEEDBACK_TIMEOUT_MS);
}

float getWheelRequestedRpm(int wheelIndex) {
  int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  return applyMotorInverse(motorIndex, requestedMotorRpm[motorIndex]);
}

float getWheelRampedRpm(int wheelIndex) {
  int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  return applyMotorInverse(motorIndex, rampedMotorRpm[motorIndex]);
}

float getWheelMeasuredRpm(int wheelIndex) {
  int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  return applyMotorInverse(motorIndex, (float)motorRawRpm[motorIndex]);
}

int16_t getWheelCurrentCommand(int wheelIndex) {
  int motorIndex = WHEEL_TO_MOTOR[wheelIndex];
  return (int16_t)applyMotorInverse(
    motorIndex,
    (float)currentCommands[motorIndex]
  );
}

void pulseLed() {
  digitalWrite(LED_PIN, HIGH);
  ledOn = true;
  ledOffAt = millis() + LED_PULSE_MS;
}

void updateLed() {
  if (ledOn && (int32_t)(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
}

// ============================================================
// IM920 utility
// ============================================================

bool isHexChar(char c) {
  return isxdigit((unsigned char)c);
}

int hexCharToInt(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

uint8_t hexByteToUint8(const String& text) {
  if (text.length() < 2) return 0;

  int high = hexCharToInt(text[0]);
  int low = hexCharToInt(text[1]);

  if (high < 0 || low < 0) return 0;
  return (uint8_t)((high << 4) | low);
}

int8_t toInt8(uint8_t value) {
  return (int8_t)value;
}

String sanitizeAsciiLine(const String& line) {
  String result = "";

  for (int i = 0; i < line.length(); i++) {
    char c = line[i];

    if (c >= 0x20 && c <= 0x7E) {
      result += c;
    }
  }

  result.trim();
  return result;
}

String collectHexChars(const String& text) {
  String result = "";

  for (int i = 0; i < text.length(); i++) {
    if (isHexChar(text[i])) {
      result += text[i];
    }
  }

  return result;
}

bool isKnownPacketType(uint8_t type) {
  // サーボパケット0x53は調整中に無視する。
  return type == 0x42 || type == 0x44 || type == 0x4A;
}

String extractPayloadHex(String line) {
  line.trim();
  if (line.length() == 0) return "";

  int colonIndex = line.indexOf(':');
  String payload = colonIndex >= 0
    ? line.substring(colonIndex + 1)
    : line;

  String hex = collectHexChars(payload);

  if (hex.length() < 2) return "";

  uint8_t type = hexByteToUint8(hex.substring(0, 2));
  return isKnownPacketType(type) ? hex : "";
}

String textToHex(const String& text) {
  String hex = "";

  for (int i = 0; i < text.length(); i++) {
    char buffer[3];
    sprintf(buffer, "%02X", (uint8_t)text[i]);
    hex += buffer;
  }

  return hex;
}

void sendCommandToIm920(const String& command) {
  IM920.print(command);
  IM920.print("\r\n");

  Serial.print("IM920 CMD -> ");
  Serial.println(command);
}

void sendTextToPi(const String& text) {
  if (!ENABLE_REPLY_TO_PI || text.length() == 0) return;

  sendCommandToIm920("TXDA " + textToHex(text));
}

// ============================================================
// CAN / C620
// ============================================================

bool setupCan() {
  twai_general_config_t generalConfig =
    TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX_PIN,
      CAN_RX_PIN,
      TWAI_MODE_NORMAL
    );

  generalConfig.tx_queue_len = 10;
  generalConfig.rx_queue_len = 64;

  twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(
    &generalConfig,
    &timingConfig,
    &filterConfig
  );

  if (result != ESP_OK) {
    Serial.print("TWAI install failed: ");
    Serial.println((int)result);
    return false;
  }

  result = twai_start();

  if (result != ESP_OK) {
    Serial.print("TWAI start failed: ");
    Serial.println((int)result);
    twai_driver_uninstall();
    return false;
  }

  Serial.println("TWAI READY: 1Mbps TX=GPIO4 RX=GPIO5");
  return true;
}

bool sendC620CurrentFrame() {
  if (!canReady) return false;

  twai_message_t message = {};
  message.identifier = C620_COMMAND_ID;
  message.data_length_code = 8;

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; motorIndex++) {
    uint16_t command = (uint16_t)currentCommands[motorIndex];

    message.data[motorIndex * 2] =
      (uint8_t)((command >> 8) & 0xFF);

    message.data[motorIndex * 2 + 1] =
      (uint8_t)(command & 0xFF);
  }

  esp_err_t result = twai_transmit(&message, 0);

  if (result != ESP_OK) {
    uint32_t now = millis();

    if (now - lastCanErrorPrintMs >= 1000) {
      lastCanErrorPrintMs = now;
      Serial.print("CAN TX failed: ");
      Serial.println((int)result);
    }

    return false;
  }

  return true;
}

void sendC620CurrentPeriodically() {
  if (!canReady) return;

  uint32_t nowUs = micros();
  if (nowUs - lastCanTxUs < CAN_TX_INTERVAL_US) return;

  lastCanTxUs = nowUs;
  sendC620CurrentFrame();
}

int motorIndexFromFeedbackId(uint32_t identifier) {
  if (identifier < C620_FEEDBACK_ID_BASE ||
      identifier >= C620_FEEDBACK_ID_BASE + NUM_MOTORS) {
    return -1;
  }

  return (int)(identifier - C620_FEEDBACK_ID_BASE);
}

void handleC620Feedback(const twai_message_t& message) {
  if (message.data_length_code < 7) return;

  int motorIndex = motorIndexFromFeedbackId(message.identifier);
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) return;

  motorRotorAngle[motorIndex] =
    ((uint16_t)message.data[0] << 8) | message.data[1];

  motorRawRpm[motorIndex] = readInt16BigEndian(
    message.data[2],
    message.data[3]
  );

  motorMeasuredCurrent[motorIndex] = readInt16BigEndian(
    message.data[4],
    message.data[5]
  );

  motorTemperature[motorIndex] = message.data[6];
  motorFeedbackValid[motorIndex] = true;
  motorFeedbackMs[motorIndex] = millis();
}

void readCanFrames() {
  if (!canReady) return;

  twai_message_t message;

  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd || message.rtr) continue;
    handleC620Feedback(message);
  }
}

void resetAllPi() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    pidIntegral[i] = 0.0f;
  }
}

void zeroMotorTargetsAndCurrent() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    requestedMotorRpm[i] = 0.0f;
    rampedMotorRpm[i] = 0.0f;
    currentCommands[i] = 0;
    pidIntegral[i] = 0.0f;
  }

  motorsActive = false;
}

void stopAllMotors(const char* reason) {
  zeroMotorTargetsAndCurrent();

  // ゼロ電流を直ちに複数回送信。
  sendC620CurrentFrame();
  delay(2);
  sendC620CurrentFrame();

  Serial.print("STOP: ");
  Serial.println(reason);
}

bool allMotorFeedbackFresh() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!feedbackFresh(i)) return false;
  }

  return true;
}

void setWheelTargets(const float wheelTargets[NUM_WHEELS]) {
  if (estopLatched) {
    stopAllMotors("ESTOP LATCHED");
    return;
  }

  bool anyActive = false;

  for (int wheelIndex = 0; wheelIndex < NUM_WHEELS; wheelIndex++) {
    int motorIndex = WHEEL_TO_MOTOR[wheelIndex];

    float wheelRpm = clampFloat(
      wheelTargets[wheelIndex],
      -(float)HARD_TARGET_RPM_LIMIT,
      (float)HARD_TARGET_RPM_LIMIT
    );

    requestedMotorRpm[motorIndex] =
      applyMotorInverse(motorIndex, wheelRpm);

    if (fabsf(wheelRpm) >= 1.0f) {
      anyActive = true;
    }
  }

  if (anyActive && !allMotorFeedbackFresh()) {
    stopAllMotors("NO C620 FEEDBACK");
    sendTextToPi("ERR NOFB");
    return;
  }

  motorsActive = anyActive;
}

// ------------------------------------------------------------
// テストモード側で実装する関数
// ------------------------------------------------------------
float getSpeedKp();
float getSpeedKi();
int16_t getRuntimeCurrentLimit();

void applyDpadMotion();
void handleTestButton(uint8_t buttonId);
void sendModeTelemetry();
void printModeSetup();
void onRadioTimeout();

// ============================================================
// PI速度制御
// ============================================================

void updateMotorSpeedControl() {
  if (!canReady) return;

  uint32_t nowUs = micros();
  uint32_t elapsedUs = nowUs - lastMotorControlUs;

  if (elapsedUs < MOTOR_CONTROL_INTERVAL_US) return;

  lastMotorControlUs = nowUs;

  float dt = (float)elapsedUs / 1000000.0f;

  if (dt <= 0.0f || dt > 0.05f) {
    dt = (float)MOTOR_CONTROL_INTERVAL_US / 1000000.0f;
  }

  float maximumRpmChange = TARGET_RPM_SLEW_PER_SEC * dt;

  for (int motorIndex = 0; motorIndex < NUM_MOTORS; motorIndex++) {
    rampedMotorRpm[motorIndex] = moveToward(
      rampedMotorRpm[motorIndex],
      requestedMotorRpm[motorIndex],
      maximumRpmChange
    );

    if (fabsf(rampedMotorRpm[motorIndex]) < 1.0f) {
      currentCommands[motorIndex] = 0;
      pidIntegral[motorIndex] = 0.0f;
      continue;
    }

    if (!feedbackFresh(motorIndex)) {
      estopLatched = true;
      stopAllMotors("FEEDBACK TIMEOUT");
      sendTextToPi("ESTOP NOFB");
      return;
    }

    if (motorTemperature[motorIndex] >= MOTOR_TEMP_STOP_C) {
      estopLatched = true;
      stopAllMotors("MOTOR OVER TEMP");
      sendTextToPi("ESTOP TEMP");
      return;
    }

    float error =
      rampedMotorRpm[motorIndex] - (float)motorRawRpm[motorIndex];

    float integralOld = pidIntegral[motorIndex];
    float integralCandidate = integralOld + error * dt;

    integralCandidate = clampFloat(
      integralCandidate,
      -PID_INTEGRAL_LIMIT,
      PID_INTEGRAL_LIMIT
    );

    float kp = getSpeedKp();
    float ki = getSpeedKi();
    float currentLimit = (float)getRuntimeCurrentLimit();

    float outputCandidate =
      kp * error + ki * integralCandidate;

    bool saturatedHigh = outputCandidate >= currentLimit;
    bool saturatedLow = outputCandidate <= -currentLimit;

    // 飽和を悪化させる方向では積分しない。
    bool allowIntegral =
      (!saturatedHigh && !saturatedLow) ||
      (saturatedHigh && error < 0.0f) ||
      (saturatedLow && error > 0.0f);

    float integralUsed =
      allowIntegral ? integralCandidate : integralOld;

    float output =
      kp * error + ki * integralUsed;

    currentCommands[motorIndex] = clampInt16(
      (int32_t)lroundf(output),
      -getRuntimeCurrentLimit(),
      getRuntimeCurrentLimit()
    );

    pidIntegral[motorIndex] = integralUsed;
  }
}

// ============================================================
// PS5 / IM920 packet handling
// ============================================================

String buttonName(uint8_t id) {
  switch (id) {
    case 0: return "CROSS";
    case 1: return "CIRCLE";
    case 2: return "TRIANGLE";
    case 3: return "SQUARE";
    case 4: return "L1";
    case 5: return "R1";
    case 6: return "L2";
    case 7: return "R2";
    case 8: return "SHARE";
    case 9: return "OPTIONS";
    case 10: return "PS";
    case 11: return "L3";
    case 12: return "R3";
    case 13: return "TOUCHPAD";
    default: return "UNKNOWN";
  }
}

void handleButtonPacket(const String& hex) {
  if (hex.length() < 6) return;

  uint8_t buttonId = hexByteToUint8(hex.substring(2, 4));
  uint8_t state = hexByteToUint8(hex.substring(4, 6));

  Serial.print("BUTTON ");
  Serial.print(buttonName(buttonId));
  Serial.print(" state=");
  Serial.println(state);

  if (state != 1) return;

  // 共通安全操作
  if (buttonId == 10) {
    estopLatched = true;
    lastDpadX = 0;
    lastDpadY = 0;
    stopAllMotors("PS ESTOP");
    sendTextToPi("ESTOP ON");
    return;
  }

  if (buttonId == 8) {
    lastDpadX = 0;
    lastDpadY = 0;
    stopAllMotors("ESTOP RESET");
    estopLatched = false;
    sendTextToPi("ESTOP OFF");
    return;
  }

  if (buttonId == 0) {
    lastDpadX = 0;
    lastDpadY = 0;
    stopAllMotors("CROSS");
    sendTextToPi("STOP");
    return;
  }

  handleTestButton(buttonId);
}

void handleDpadPacket(const String& hex) {
  if (hex.length() < 6) return;

  uint8_t axis = hexByteToUint8(hex.substring(2, 4));
  int8_t value = toInt8(hexByteToUint8(hex.substring(4, 6)));

  if (axis == 0) {
    lastDpadX = value;
  } else if (axis == 1) {
    lastDpadY = value;
  } else {
    return;
  }

  applyDpadMotion();
}

void handleJoyPacket(const String& hex) {
  if (hex.length() < 18) return;

  // スティック値は意図的に使用しない。
  lastDpadX = toInt8(hexByteToUint8(hex.substring(14, 16)));
  lastDpadY = toInt8(hexByteToUint8(hex.substring(16, 18)));

  applyDpadMotion();
}

void handlePayloadHex(const String& hex) {
  if (hex.length() < 2) return;

  uint8_t type = hexByteToUint8(hex.substring(0, 2));
  if (!isKnownPacketType(type)) return;

  lastRadioRxMs = millis();
  pulseLed();

  if (type == 0x42) {
    handleButtonPacket(hex);
  } else if (type == 0x44) {
    handleDpadPacket(hex);
  } else if (type == 0x4A) {
    handleJoyPacket(hex);
  }
}

void handleIm920Line(String rawLine) {
  String line = sanitizeAsciiLine(rawLine);
  if (line.length() == 0) return;

  // IM920自身のコマンド応答は無視。
  if (line == "OK" ||
      line == "NG" ||
      line.startsWith("IM920")) {
    return;
  }

  String hex = extractPayloadHex(line);

  if (hex.length() > 0) {
    handlePayloadHex(hex);
  }
}

void readIm920Serial() {
  while (IM920.available()) {
    char c = (char)IM920.read();

    if (c == '\r' || c == '\n') {
      imLine.trim();

      if (imLine.length() > 0) {
        handleIm920Line(imLine);
        imLine = "";
      }

      continue;
    }

    imLine += c;

    if (imLine.length() > 180) {
      imLine = "";
    }
  }
}

void checkRadioTimeout() {
  if (millis() - lastRadioRxMs <= RADIO_TIMEOUT_MS) return;

  if (motorsActive) {
    stopAllMotors("RADIO TIMEOUT");
    onRadioTimeout();
  }

  lastRadioRxMs = millis();
}

// ============================================================
// setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  servoDriver.begin();
  servoDriver.setPWMFreq(50);
  delay(10);
  disableAllServoOutputs();

  canReady = setupCan();

  if (!canReady) {
    Serial.println("WARNING: CAN initialization failed.");
  }

  IM920.begin(19200, SERIAL_8N1, IM920_RX, IM920_TX);
  delay(1000);

  Serial.println();
  Serial.println("================================================");
  printModeSetup();
  Serial.println("================================================");
  Serial.println("CAN: GPIO4=TX GPIO5=RX 1Mbps");
  Serial.println("FL=ID1 FR=ID3 RL=ID2 RR=ID4");
  Serial.println("Stick input is disabled. Use DPAD only.");
  Serial.println("PCA9685 CH0-CH15 are forced FULL OFF.");
  Serial.println("CROSS=STOP PS=ESTOP SHARE=ESTOP RESET");
  Serial.println();

  sendCommandToIm920("RDID");
  delay(100);
  sendCommandToIm920("RDNN");
  delay(100);
  sendCommandToIm920("RDGN");
  delay(100);
  sendCommandToIm920("RDCH");

  zeroMotorTargetsAndCurrent();
  lastRadioRxMs = millis();
  lastMotorControlUs = micros();
  lastCanTxUs = micros();

  sendC620CurrentFrame();
  sendTextToPi("TEST READY");

  Serial.println("READY");
}

void loop() {
  updateLed();
  readIm920Serial();

  readCanFrames();
  updateMotorSpeedControl();
  sendC620CurrentPeriodically();

  checkRadioTimeout();
  sendModeTelemetry();
}

// ============================================================
// 車輪ゲイン調整モード
// ============================================================

// PI調整コードで決定した値へ変更する。
static const float FIXED_SPEED_KP = 1.00f;
static const float FIXED_SPEED_KI = 0.20f;

static const int16_t FIXED_CURRENT_LIMIT = 3000;

enum MotionMode {
  MOTION_STOP,
  MOTION_FWD,
  MOTION_BWD,
  MOTION_RIGHT,
  MOTION_LEFT
};

enum GainBank {
  GAIN_FWD,
  GAIN_BWD,
  GAIN_RIGHT,
  GAIN_LEFT
};

MotionMode activeMotion = MOTION_STOP;
GainBank selectedGainBank = GAIN_FWD;

int16_t testRpm = 1200;
static const int16_t TEST_RPM_MIN = 400;
static const int16_t TEST_RPM_MAX = 4000;
static const int16_t TEST_RPM_STEP = 200;

static const float GAIN_STEP = 0.005f;
static const float GAIN_MIN = 0.800f;
static const float GAIN_MAX = 1.200f;

uint8_t selectedWheel = 0;

// 最初はすべて1.000から調整する。
float wheelGainFwd[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

float wheelGainBwd[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

float wheelGainRight[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

float wheelGainLeft[NUM_WHEELS] = {
  1.000f, 1.000f, 1.000f, 1.000f
};

bool gainDumpPending = false;
uint8_t gainDumpIndex = 0;
uint32_t lastGainDumpMs = 0;

float getSpeedKp() {
  return FIXED_SPEED_KP;
}

float getSpeedKi() {
  return FIXED_SPEED_KI;
}

int16_t getRuntimeCurrentLimit() {
  return FIXED_CURRENT_LIMIT;
}

const char* motionName(MotionMode mode) {
  switch (mode) {
    case MOTION_FWD: return "FWD";
    case MOTION_BWD: return "BWD";
    case MOTION_RIGHT: return "RIGHT";
    case MOTION_LEFT: return "LEFT";
    default: return "STOP";
  }
}

const char* gainBankName(GainBank bank) {
  switch (bank) {
    case GAIN_FWD: return "FWD";
    case GAIN_BWD: return "BWD";
    case GAIN_RIGHT: return "RIGHT";
    case GAIN_LEFT: return "LEFT";
    default: return "?";
  }
}

char gainBankCode(GainBank bank) {
  switch (bank) {
    case GAIN_FWD: return 'F';
    case GAIN_BWD: return 'B';
    case GAIN_RIGHT: return 'R';
    case GAIN_LEFT: return 'L';
    default: return '?';
  }
}

float* gainArray(GainBank bank) {
  switch (bank) {
    case GAIN_FWD: return wheelGainFwd;
    case GAIN_BWD: return wheelGainBwd;
    case GAIN_RIGHT: return wheelGainRight;
    case GAIN_LEFT: return wheelGainLeft;
    default: return wheelGainFwd;
  }
}

MotionMode readDpadMotion() {
  if (lastDpadY < 0) return MOTION_FWD;
  if (lastDpadY > 0) return MOTION_BWD;
  if (lastDpadX > 0) return MOTION_RIGHT;
  if (lastDpadX < 0) return MOTION_LEFT;
  return MOTION_STOP;
}

GainBank gainBankFromMotion(MotionMode mode) {
  switch (mode) {
    case MOTION_BWD: return GAIN_BWD;
    case MOTION_RIGHT: return GAIN_RIGHT;
    case MOTION_LEFT: return GAIN_LEFT;
    case MOTION_FWD:
    default:
      return GAIN_FWD;
  }
}

void makeGainTestWheelTargets(
  MotionMode mode,
  float wheelTargets[NUM_WHEELS]
) {
  for (int i = 0; i < NUM_WHEELS; i++) {
    wheelTargets[i] = 0.0f;
  }

  GainBank bank = gainBankFromMotion(mode);
  float* gains = gainArray(bank);

  switch (mode) {
    case MOTION_FWD:
      for (int i = 0; i < NUM_WHEELS; i++) {
        wheelTargets[i] = (float)testRpm * gains[i];
      }
      break;

    case MOTION_BWD:
      for (int i = 0; i < NUM_WHEELS; i++) {
        wheelTargets[i] = -(float)testRpm * gains[i];
      }
      break;

    case MOTION_RIGHT:
      for (int i = 0; i < NUM_WHEELS; i++) {
        wheelTargets[i] =
          (float)STRAFE_SIGN[i] *
          (float)testRpm *
          gains[i];
      }
      break;

    case MOTION_LEFT:
      for (int i = 0; i < NUM_WHEELS; i++) {
        wheelTargets[i] =
          -(float)STRAFE_SIGN[i] *
          (float)testRpm *
          gains[i];
      }
      break;

    default:
      break;
  }
}

void applyDpadMotion() {
  MotionMode requestedMotion = readDpadMotion();

  if (requestedMotion == MOTION_STOP) {
    if (activeMotion != MOTION_STOP || motorsActive) {
      activeMotion = MOTION_STOP;
      stopAllMotors("DPAD RELEASE");
    }
    return;
  }

  if (estopLatched) {
    stopAllMotors("ESTOP LATCHED");
    return;
  }

  activeMotion = requestedMotion;
  selectedGainBank = gainBankFromMotion(activeMotion);

  float wheelTargets[NUM_WHEELS];
  makeGainTestWheelTargets(activeMotion, wheelTargets);
  setWheelTargets(wheelTargets);
}

void sendSelectedGain() {
  float* gains = gainArray(selectedGainBank);

  String message = "G ";
  message += gainBankCode(selectedGainBank);
  message += " ";
  message += WHEEL_NAME[selectedWheel];
  message += "=";
  message += String(
    (int)lroundf(gains[selectedWheel] * 1000.0f)
  );

  sendTextToPi(message);

  Serial.print("GAIN ");
  Serial.print(gainBankName(selectedGainBank));
  Serial.print(" ");
  Serial.print(WHEEL_NAME[selectedWheel]);
  Serial.print(" = ");
  Serial.println(gains[selectedWheel], 3);
}

void requestGainDump() {
  gainDumpPending = true;
  gainDumpIndex = 0;
  lastGainDumpMs = 0;
}

void sendGainDumpStep() {
  if (!gainDumpPending) return;

  uint32_t now = millis();
  if (now - lastGainDumpMs < 300) return;

  lastGainDumpMs = now;

  GainBank bank = (GainBank)gainDumpIndex;
  float* gains = gainArray(bank);

  String message = "G";
  message += gainBankCode(bank);

  for (int i = 0; i < NUM_WHEELS; i++) {
    message += " ";
    message += String((int)lroundf(gains[i] * 1000.0f));
  }

  sendTextToPi(message);

  Serial.print("GAIN ARRAY ");
  Serial.print(gainBankName(bank));
  Serial.print(" = {");

  for (int i = 0; i < NUM_WHEELS; i++) {
    Serial.print(gains[i], 3);
    if (i < NUM_WHEELS - 1) Serial.print(", ");
  }

  Serial.println("}");

  gainDumpIndex++;

  if (gainDumpIndex >= 4) {
    gainDumpPending = false;
  }
}

void changeSelectedGain(float delta) {
  float* gains = gainArray(selectedGainBank);

  gains[selectedWheel] = clampFloat(
    gains[selectedWheel] + delta,
    GAIN_MIN,
    GAIN_MAX
  );

  resetAllPi();

  // 十字キーを押したままなら即時反映。
  applyDpadMotion();
  sendSelectedGain();
}

void handleTestButton(uint8_t buttonId) {
  switch (buttonId) {
    case 4:  // L1: rpm down
      testRpm = clampInt16(
        testRpm - TEST_RPM_STEP,
        TEST_RPM_MIN,
        TEST_RPM_MAX
      );
      applyDpadMotion();
      sendSelectedGain();
      break;

    case 5:  // R1: rpm up
      testRpm = clampInt16(
        testRpm + TEST_RPM_STEP,
        TEST_RPM_MIN,
        TEST_RPM_MAX
      );
      applyDpadMotion();
      sendSelectedGain();
      break;

    case 6:  // L2: previous wheel
      selectedWheel =
        (selectedWheel + NUM_WHEELS - 1) % NUM_WHEELS;
      sendSelectedGain();
      break;

    case 7:  // R2: next wheel
      selectedWheel =
        (selectedWheel + 1) % NUM_WHEELS;
      sendSelectedGain();
      break;

    case 3:  // SQUARE: gain down
      changeSelectedGain(-GAIN_STEP);
      break;

    case 2:  // TRIANGLE: gain up
      changeSelectedGain(+GAIN_STEP);
      break;

    case 1:  // CIRCLE: selected status
      sendSelectedGain();
      break;

    case 9:  // OPTIONS: dump all gains
      requestGainDump();
      break;

    case 13: {  // TOUCHPAD: selected gain reset
      float* gains = gainArray(selectedGainBank);
      gains[selectedWheel] = 1.000f;
      resetAllPi();
      applyDpadMotion();
      sendSelectedGain();
      break;
    }

    default:
      break;
  }
}

void sendModeTelemetry() {
  sendGainDumpStep();

  uint32_t now = millis();
  if (now - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;

  lastTelemetryMs = now;

  int wheelIndex = telemetryWheelIndex;
  telemetryWheelIndex =
    (telemetryWheelIndex + 1) % NUM_WHEELS;

  String message = "W";
  message += WHEEL_NAME[wheelIndex];
  message += " T";
  message += String((int)lroundf(getWheelRampedRpm(wheelIndex)));
  message += " R";
  message += String((int)lroundf(getWheelMeasuredRpm(wheelIndex)));
  message += " C";
  message += String(getWheelCurrentCommand(wheelIndex));

  sendTextToPi(message);

  Serial.print("GAIN TEST ");
  Serial.print(motionName(activeMotion));
  Serial.print(" ");
  Serial.print(WHEEL_NAME[wheelIndex]);
  Serial.print(" target=");
  Serial.print(getWheelRampedRpm(wheelIndex), 1);
  Serial.print(" rpm=");
  Serial.print(getWheelMeasuredRpm(wheelIndex), 1);
  Serial.print(" cmd=");
  Serial.print(getWheelCurrentCommand(wheelIndex));
  Serial.print(" selected=");
  Serial.print(gainBankName(selectedGainBank));
  Serial.print("/");
  Serial.println(WHEEL_NAME[selectedWheel]);
}

void onRadioTimeout() {
  activeMotion = MOTION_STOP;
  lastDpadX = 0;
  lastDpadY = 0;
  sendTextToPi("STOP RADIO");
}

void printModeSetup() {
  Serial.println("IM920 + PS5 DPAD WHEEL GAIN TUNING TEST");
  Serial.println();
  Serial.println("Edit FIXED_SPEED_KP / FIXED_SPEED_KI first.");
  Serial.println("All wheel gains start at 1.000.");
  Serial.println();
  Serial.println("DPAD UP/DOWN   : select FWD/BWD bank and move");
  Serial.println("DPAD RIGHT/LEFT: select RIGHT/LEFT bank and move");
  Serial.println("L1/R1          : test rpm -/+200");
  Serial.println("L2/R2          : previous/next wheel");
  Serial.println("SQUARE/TRIANGLE: selected gain -/+0.005");
  Serial.println("CIRCLE         : send selected gain");
  Serial.println("OPTIONS        : send all 4 gain arrays");
  Serial.println("TOUCHPAD       : reset selected gain to 1.000");
  Serial.println();
  Serial.print("Fixed Kp=");
  Serial.print(FIXED_SPEED_KP, 3);
  Serial.print(" Ki=");
  Serial.print(FIXED_SPEED_KI, 3);
  Serial.print(" rpm=");
  Serial.println(testRpm);
}
