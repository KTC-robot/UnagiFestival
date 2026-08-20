#include "im920/im920.hpp"

#include <ctype.h>
#include <string_view>

#include "air_cylinder/air_cylinder_ctrl.hpp"
#include "chassis_ctrl/chassis_ctrl.hpp"
#include "command/constants.hpp"
#include "command/decoder.hpp"
#include "command/dispatcher.hpp"
#include "device/md20a_driver.hpp"
#include "im920/constants.hpp"

using namespace CommandProtocol;
using namespace Im920Config;

namespace {
/** @brief UARTで受けた1行から表示可能なASCII文字だけを残す。 */
String sanitizeAsciiLine(const String& line) {
  String cleaned;
  for (int index = 0; index < line.length(); ++index) {
    const char value = line[index];
    if (value >= 0x20 && value <= 0x7E) cleaned += value;
  }
  cleaned.trim();
  return cleaned;
}

String collectHexChars(const String& text) {
  String hex;
  for (int index = 0; index < text.length(); ++index) {
    if (isxdigit(static_cast<unsigned char>(text[index]))) hex += text[index];
  }
  return hex;
}

String textToHex(const String& text) {
  String hex;
  for (int index = 0; index < text.length(); ++index) {
    char buffer[3];
    snprintf(buffer, sizeof(buffer), "%02X", static_cast<uint8_t>(text[index]));
    hex += buffer;
  }
  return hex;
}

HardwareSerial IM920(2);
String imLine;
String pcLine;
bool ledOn = false;
uint32_t ledOffAt = 0;
uint32_t lastRxMs = 0;
uint32_t driveCommandCount = 0;

/**
 * @brief Gain Tuning結果1件を確実に届けるstop-and-wait送信状態。
 *
 * ローカルmoduleの受理応答と、Raspberry Piから返るapplication ACKを
 * 区別するために状態を分けて管理する。
 */
enum class GainTuningTxState : uint8_t {
  IDLE,                 ///< 送信対象がなく待機している。
  WAIT_LOCAL_RESPONSE,  ///< ESP32側IM920のOK/NGを待っている。
  WAIT_REMOTE_ACK,      ///< Raspberry Piから対応するACKを待っている。
  TURNAROUND_GUARD,     ///< 無線の送受信方向が切り替わるまで待っている。
};

int tuningTxIndex = -1;
GainTuningTxState tuningTxState = GainTuningTxState::IDLE;
uint8_t tuningTxAttemptCount = 0;
uint32_t tuningTxStartedMs = 0;
uint32_t tuningTxAllowedMs = 0;

void resetGainTuningTxState() {
  tuningTxIndex = -1;
  tuningTxState = GainTuningTxState::IDLE;
  tuningTxAttemptCount = 0;
  tuningTxStartedMs = 0;
  tuningTxAllowedMs = 0;
}

String gainTuningResultLabel(int index) {
  return index < GAIN_TUNING_WHEEL_COUNT ? String("WG") + String(index) : "WD";
}

void failGainTuningResultTx() {
  Serial.print("[IM920] Gain Tuning結果のACK待機が上限に達しました packet=");
  Serial.println(gainTuningResultLabel(tuningTxIndex));
  chassisCtrlClearGainTuningResultReady();
  resetGainTuningTxState();
}

void scheduleGainTuningResultRetry() {
  if (tuningTxAttemptCount > GAIN_TUNING_RESULT_MAX_RETRIES) {
    failGainTuningResultTx();
    return;
  }
  tuningTxState = GainTuningTxState::TURNAROUND_GUARD;
  tuningTxAllowedMs = millis() + GAIN_TUNING_RESULT_TURNAROUND_GUARD_MS;
}

/** @brief IM920固有lineからCommand protocolの16進payloadだけを抽出する。 */
String extractPayloadHex(String line) {
  line.trim();
  if (line.length() == 0) return "";

  const int colonIndex = line.indexOf(':');
  const String payload = colonIndex >= 0 ? line.substring(colonIndex + 1) : line;
  const String hex = collectHexChars(payload);
  return hex.length() >= 2 ? hex : "";
}

void pulseLed() {
  digitalWrite(LED_PIN, HIGH);
  ledOn = true;
  ledOffAt = millis() + LED_PULSE_MS;
}

void updateLed() {
  if (ledOn && static_cast<int32_t>(millis() - ledOffAt) >= 0) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }
}

void sendCommandToIm920(const String& command) {
  IM920.print(command);
  IM920.print("\r\n");
  Serial.print("[IM920] command送信 -> ");
  Serial.println(command);
}

void flushIm920Input() {
  while (IM920.available()) IM920.read();
}

String readIm920LineBlocking(uint32_t timeoutMs) {
  String line;
  const uint32_t startedMs = millis();
  while (millis() - startedMs < timeoutMs) {
    while (IM920.available()) {
      const char c = static_cast<char>(IM920.read());
      if (c == '\r' || c == '\n') {
        line.trim();
        if (line.length() > 0) return sanitizeAsciiLine(line);
        line = "";
        continue;
      }
      line += c;
      if (line.length() > 180) line = "";
    }
    delay(1);
  }
  return "";
}

String queryIm920Setting(const char* command) {
  flushIm920Input();
  sendCommandToIm920(command);
  const String response = readIm920LineBlocking(IM920_QUERY_TIMEOUT_MS);
  Serial.print("[IM920] 設定応答 ");
  Serial.print(command);
  Serial.print(" <- ");
  Serial.println(response.length() > 0 ? response : "<応答なし>");
  return response;
}

void printIm920Configuration() {
  Serial.println("[IM920] 設定確認を開始します");
  const String version = queryIm920Setting("RDVR");
  const String id = queryIm920Setting("RDID");
  const String node = queryIm920Setting("RDNN");
  const String group = queryIm920Setting("RDGN");
  const String channel = queryIm920Setting("RDCH");
  (void)version;
  (void)id;
  (void)node;

  if (group.length() > 0 && group != EXPECTED_IM920_GROUP) {
    Serial.print("[IM920] 警告: groupが設定値と一致しません 期待値=");
    Serial.print(EXPECTED_IM920_GROUP);
    Serial.print(" 実際=");
    Serial.println(group);
  }
  if (channel.length() > 0 && channel != EXPECTED_IM920_CHANNEL) {
    Serial.print("[IM920] 警告: channelが設定値と一致しません 期待値=");
    Serial.print(EXPECTED_IM920_CHANNEL);
    Serial.print(" 実際=");
    Serial.println(channel);
  }
  Serial.println("[IM920] 設定確認が完了しました");
}

void handleGainTuningResultAck(uint8_t ackIndex) {
  if (tuningTxState != GainTuningTxState::WAIT_REMOTE_ACK ||
      ackIndex != tuningTxIndex) return;

  Serial.print("[IM920] Gain Tuning結果ACK受信 ");
  Serial.println(gainTuningResultLabel(ackIndex));
  if (ackIndex == GAIN_TUNING_WHEEL_COUNT) {
    chassisCtrlClearGainTuningResultReady();
    resetGainTuningTxState();
    return;
  }

  ++tuningTxIndex;
  tuningTxAttemptCount = 0;
  tuningTxState = GainTuningTxState::TURNAROUND_GUARD;
  tuningTxAllowedMs = millis() + GAIN_TUNING_RESULT_TURNAROUND_GUARD_MS;
}

void sendDispatchReply(const CommandDispatchResult& result) {
  switch (result.reply) {
    case CommandReply::CTRL_STOP:
      im920SendText("CTRL STOP");
      break;
    case CommandReply::CTRL_ESTOP:
      Serial.println("[IM920] 緊急停止Commandを実行しました");
      im920SendText("CTRL ESTOP");
      break;
    case CommandReply::WHEEL_GAIN: {
      String reply = "WGS,";
      reply += String(result.wheelGainDirection);
      reply += ",";
      reply += String(result.wheelGainIndex);
      reply += ",";
      reply += String(result.wheelGain, 3);
      im920SendText(reply);
      break;
    }
    case CommandReply::TUNE_START:
      im920SendText("TUNE START");
      break;
    case CommandReply::STEP_RESET:
      im920SendText("STEP RESET");
      break;
    case CommandReply::NONE:
      break;
  }
}

void handlePayloadHex(const String& payloadHex) {
  // IM920固有headerを除去したpayloadだけをDecoderへ渡す。
  // decodeと実行に成功したpacketのみを有効な通信として扱う。
  Command command;
  const std::string_view payload(payloadHex.c_str(), payloadHex.length());
  if (!decodeCommand(payload, command)) return;

  const CommandDispatchResult result = dispatchCommand(command);
  if (!result.executed) return;

  // timeoutとLEDは、decodeと実行に成功したapplication Commandだけで更新する。
  lastRxMs = millis();
  pulseLed();

  if (result.resetGainTuningTx) resetGainTuningTxState();
  if (result.gainTuningResultAck) {
    handleGainTuningResultAck(result.gainTuningResultIndex);
  }
  if (result.driveExecuted) {
    ++driveCommandCount;
    if (driveCommandCount % DRIVE_ACK_INTERVAL == 0) {
      im920SendText(String("DRIVE OK PWR=") + String(chassisCtrlGetPowerPercent()));
    }
  }
  sendDispatchReply(result);
}

void sendGainTuningResultsIfReady() {
  if (tuningTxIndex < 0) {
    if (!chassisCtrlGainTuningResultReady()) return;
    tuningTxIndex = 0;
    tuningTxAttemptCount = 0;
    tuningTxState = GainTuningTxState::TURNAROUND_GUARD;
    tuningTxAllowedMs = millis();
  }

  if (tuningTxState == GainTuningTxState::WAIT_LOCAL_RESPONSE) {
    if (millis() - tuningTxStartedMs < GAIN_TUNING_TX_RESPONSE_TIMEOUT_MS) return;
    Serial.print("[IM920] ローカル応答待ちがtimeoutしました ");
    Serial.println(gainTuningResultLabel(tuningTxIndex));
    scheduleGainTuningResultRetry();
    return;
  }
  if (tuningTxState == GainTuningTxState::WAIT_REMOTE_ACK) {
    if (millis() - tuningTxStartedMs < GAIN_TUNING_RESULT_ACK_TIMEOUT_MS) return;
    Serial.print("[IM920] 遠端ACK待ちがtimeoutしました ");
    Serial.println(gainTuningResultLabel(tuningTxIndex));
    scheduleGainTuningResultRetry();
    return;
  }
  if (tuningTxState != GainTuningTxState::TURNAROUND_GUARD ||
      static_cast<int32_t>(millis() - tuningTxAllowedMs) < 0) return;
  if (tuningTxAttemptCount > GAIN_TUNING_RESULT_MAX_RETRIES) {
    failGainTuningResultTx();
    return;
  }

  String message;
  if (tuningTxIndex < GAIN_TUNING_WHEEL_COUNT) {
    const ChassisGainTuningResult result =
      chassisCtrlGetGainTuningResult(tuningTxIndex);
    message = String("WG") + String(tuningTxIndex) + ",";
    message += String(static_cast<int>(lroundf(result.meanAbsoluteRpm)));
    message += ",";
    message += String(result.sampleCount);
    message += ",";
    message += String(static_cast<int>(lroundf(result.standardDeviationRpm)));
  } else if (tuningTxIndex == GAIN_TUNING_WHEEL_COUNT) {
    message = "WD";
  } else {
    return;
  }

  ++tuningTxAttemptCount;
  Serial.print("[IM920] Gain Tuning結果送信 ");
  Serial.print(gainTuningResultLabel(tuningTxIndex));
  Serial.print(" 試行回数=");
  Serial.println(tuningTxAttemptCount);
  if (ENABLE_GAIN_TUNING_TX_LOG) {
    Serial.print("[IM920][DEBUG] 送信text=");
    Serial.println(message);
    Serial.print("[IM920][DEBUG] text長[byte]=");
    Serial.println(message.length());
    Serial.print("[IM920][DEBUG] hex長[文字]=");
    Serial.println(message.length() * 2);
    Serial.print("[IM920][DEBUG] TXDA command長[文字]=");
    Serial.println(5 + message.length() * 2);
    Serial.print("[IM920][DEBUG] 結果index=");
    Serial.println(tuningTxIndex);
  }
  im920SendText(message);
  tuningTxState = GainTuningTxState::WAIT_LOCAL_RESPONSE;
  tuningTxStartedMs = millis();
}

void handleIm920Line(const String& rawLine) {
  const String line = sanitizeAsciiLine(rawLine);
  if (line.length() == 0) return;
  if (SHOW_RAW) {
    // Serial.print("[IM920][DEBUG] raw受信 <- ");
    // Serial.println(line);
  }

  // OK/NGはローカルIM920の応答であり、application Commandではない。
  if (line == "OK") {
    if (tuningTxState == GainTuningTxState::WAIT_LOCAL_RESPONSE) {
      tuningTxState = GainTuningTxState::WAIT_REMOTE_ACK;
      tuningTxStartedMs = millis();
      // OKはローカルIM920がTXDAを受理したことだけを示す。
      // Raspberry Piへの配送完了は、別途remote ACKで確認する。
      Serial.print("[IM920] ローカルIM920がTXDAを受理しました ");
      Serial.println(gainTuningResultLabel(tuningTxIndex));
    }
    return;
  }
  if (line == "NG") {
    if (tuningTxState == GainTuningTxState::WAIT_LOCAL_RESPONSE) {
      Serial.print("[IM920] ローカルIM920がTXDAを拒否しました ");
      Serial.println(gainTuningResultLabel(tuningTxIndex));
      scheduleGainTuningResultRetry();
    }
    return;
  }
  if (line.startsWith("IM920")) return;

  const String payloadHex = extractPayloadHex(line);
  if (payloadHex.length() > 0) handlePayloadHex(payloadHex);
}

void readIm920Serial() {
  // UARTのCR/LFまでを1lineとして組み立て、IM920固有line処理へ渡す。
  while (IM920.available()) {
    const char c = static_cast<char>(IM920.read());
    if (c == '\r' || c == '\n') {
      imLine.trim();
      if (imLine.length() > 0) {
        handleIm920Line(imLine);
        imLine = "";
      }
      continue;
    }
    imLine += c;
    if (imLine.length() > 180) imLine = "";
  }
}

void readPcSerialCommand() {
  // 開発時にPCから入力したIM920 commandを、そのままUARTへ中継する。
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      pcLine.trim();
      if (pcLine.length() > 0) {
        sendCommandToIm920(pcLine);
        pcLine = "";
      }
      continue;
    }
    pcLine += c;
    if (pcLine.length() > 120) pcLine = "";
  }
}
}  // namespace

void im920Begin() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  IM920.begin(IM920_BAUD, SERIAL_8N1, IM920_RX, IM920_TX);
  delay(1000);
  printIm920Configuration();
  lastRxMs = millis();
  Serial.println("[IM920] UARTの初期化が完了しました RX=16 TX=17 19200bps");
}

void im920Update() {
  updateLed();
  readIm920Serial();
  readPcSerialCommand();
  sendGainTuningResultsIfReady();
}

void im920CheckTimeout() {
  if (millis() - lastRxMs <= COMM_TIMEOUT_MS) return;

  if (tuningTxState != GainTuningTxState::IDLE) {
    resetGainTuningTxState();
    chassisCtrlClearGainTuningResultReady();
  }
  const bool unsafeState = chassisCtrlIsActive() || airCylinderCtrlActive() ||
    md20aDriverGetState() != Md20aState::STOPPED;
  if (unsafeState) {
    Serial.println("[IM920] 通信timeoutのため出力を安全停止します");
  }
  chassisCtrlStop();
  airCylinderCtrlStop();
  md20aDriverSetState(Md20aState::STOPPED);
  lastRxMs = millis();
}

void im920SendText(const String& text) {
  if (!ENABLE_REPLY_TO_PI || text.length() == 0) return;
  if (text.length() > IM920_TXDA_MAX_PAYLOAD_BYTES) {
    Serial.print("[IM920] TXDA payloadが上限を超えています length[byte]=");
    Serial.println(text.length());
    return;
  }
  sendCommandToIm920("TXDA " + textToHex(text));
}
