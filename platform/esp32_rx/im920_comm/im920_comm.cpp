#include "im920_comm.h"

#include "can_comm/can_comm.h"
#include "chassis_ctrl/chassis_ctrl.h"
#include "servo_ctrl/servo_ctrl.h"
#include "stepper_ctrl/stepper_ctrl.hpp"
#include "util/util.h"
#include "im920_comm/constants.h"

using namespace CanConfig_im920_comm;

namespace {
HardwareSerial IM920(2);

String imLine;
String pcLine;

bool ledOn = false;
uint32_t ledOffAt = 0;

uint32_t lastRxMs = 0;
uint32_t driveCommandCount = 0;

/**
 * @brief gain tuning結果1件のstop-and-wait送信状態。
 */
enum class GainTuningTxState : uint8_t {
  IDLE,                 ///< 未送信結果なし。
  WAIT_LOCAL_RESPONSE,  ///< ローカルIM920のOK/NG待ち。
  WAIT_REMOTE_ACK,      ///< Raspberry Piのapplication ACK待ち。
  TURNAROUND_GUARD      ///< 無線の送受信切替待ち。
};

int tuningTxIndex = -1;
GainTuningTxState tuningTxState = GainTuningTxState::IDLE;
uint8_t tuningTxAttemptCount = 0;
uint32_t tuningTxStartedMs = 0;
uint32_t tuningTxAllowedMs = 0;

/**
 * @brief wheel gain調整結果の逐次送信状態を初期状態へ戻す。
 *
 * STOPや新規試験を跨いで旧試験のWG/WDを送らないために使用する。
 */
void resetGainTuningTxState() {
  tuningTxIndex = -1;
  tuningTxState = GainTuningTxState::IDLE;
  tuningTxAttemptCount = 0;
  tuningTxStartedMs = 0;
  tuningTxAllowedMs = 0;
}

String gainTuningResultLabel(int index) {
  return index < GAIN_TUNING_WHEEL_COUNT
    ? String("WG") + String(index)
    : "WD";
}

void failGainTuningResultTx() {
  Serial.print("GAIN TUNING result ACK timeout packet=");
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

uint16_t parseUint16(const String& hex, int offset) {
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(
      utilHexByteToUint8(hex.substring(offset, offset + 2))
    ) << 8) |
    utilHexByteToUint8(hex.substring(offset + 2, offset + 4))
  );
}

bool isKnownPacketType(uint8_t type) {
  return (
    type == static_cast<uint8_t>(PacketType::CONTROL) ||
    type == static_cast<uint8_t>(PacketType::SERVO_SET) ||
    type == static_cast<uint8_t>(PacketType::STEPPER)
  );
}

String extractPayloadHex(String line) {
  line.trim();

  if (line.length() == 0) {
    return "";
  }

  const int colonIndex = line.indexOf(':');

  if (colonIndex >= 0) {
    const String payload = line.substring(colonIndex + 1);
    const String hex = utilCollectHexChars(payload);

    if (hex.length() >= 2) {
      const uint8_t type =
        utilHexByteToUint8(hex.substring(0, 2));

      if (isKnownPacketType(type)) {
        return hex;
      }
    }

    return "";
  }

  const String hex = utilCollectHexChars(line);

  if (hex.length() >= 2) {
    const uint8_t type =
      utilHexByteToUint8(hex.substring(0, 2));

    if (isKnownPacketType(type)) {
      return hex;
    }
  }

  return "";
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

  Serial.print("CMD -> ");
  Serial.println(command);
}

void flushIm920Input() {
  while (IM920.available()) {
    IM920.read();
  }
}

String readIm920LineBlocking(uint32_t timeoutMs) {
  String line;
  const uint32_t startedMs = millis();

  while (millis() - startedMs < timeoutMs) {
    while (IM920.available()) {
      const char c = static_cast<char>(IM920.read());

      if (c == '\r' || c == '\n') {
        line.trim();

        if (line.length() > 0) {
          return utilSanitizeAsciiLine(line);
        }

        line = "";
        continue;
      }

      line += c;

      if (line.length() > 180) {
        line = "";
      }
    }

    delay(1);
  }

  return "";
}

String queryIm920Setting(const char* command) {
  flushIm920Input();
  sendCommandToIm920(command);

  const String response = readIm920LineBlocking(IM920_QUERY_TIMEOUT_MS);

  Serial.print("IM920 CFG ");
  Serial.print(command);
  Serial.print(" <- ");
  Serial.println(response.length() > 0 ? response : "<NO RESPONSE>");

  return response;
}

void printIm920Configuration() {
  Serial.println("--- IM920 configuration check ---");

  const String version = queryIm920Setting("RDVR");
  const String id = queryIm920Setting("RDID");
  const String node = queryIm920Setting("RDNN");
  const String group = queryIm920Setting("RDGN");
  const String channel = queryIm920Setting("RDCH");

  (void)version;
  (void)id;
  (void)node;

  if (group.length() > 0 && group != EXPECTED_IM920_GROUP) {
    Serial.print("WARNING: IM920 group mismatch. expected=");
    Serial.print(EXPECTED_IM920_GROUP);
    Serial.print(" actual=");
    Serial.println(group);
  }

  if (channel.length() > 0 && channel != EXPECTED_IM920_CHANNEL) {
    Serial.print("WARNING: IM920 channel mismatch. expected=");
    Serial.print(EXPECTED_IM920_CHANNEL);
    Serial.print(" actual=");
    Serial.println(channel);
  }

  Serial.println("--- IM920 configuration check end ---");
}

void handleControlPacket(const String& hex) {
  if (hex.length() < 4) {
    return;
  }

  const ControlCommand command = static_cast<ControlCommand>(
    utilHexByteToUint8(hex.substring(2, 4))
  );

  if (command != ControlCommand::GAIN_TUNE_KEEPALIVE) {
    Serial.print("CONTROL <- ID=");
    Serial.println(static_cast<uint8_t>(command));
  }

  switch (command) {
    case ControlCommand::STOP:
      resetGainTuningTxState();
      chassisCtrlStop();
      stepperCtrlStop();
      im920CommSendText("CTRL STOP");
      break;

    case ControlCommand::EMERGENCY_STOP:
      Serial.println("EMERGENCY STOP");
      resetGainTuningTxState();
      chassisCtrlStop();
      stepperCtrlStop();
      im920CommSendText("CTRL ESTOP");
      break;

    case ControlCommand::CHANGE_POWER: {
      if (hex.length() < 6) {
        return;
      }

      const int8_t delta =
        utilToInt8(utilHexByteToUint8(hex.substring(4, 6)));

      chassisCtrlChangePower(delta);
      String reply = "CTRL PWR=";
      reply += String(chassisCtrlGetPowerPercent());
      im920CommSendText(reply);
      break;
    }

    case ControlCommand::DRIVE: {
      if (hex.length() < 10) {
        return;
      }

      const int8_t vx =
        utilToInt8(utilHexByteToUint8(hex.substring(4, 6)));
      const int8_t vy =
        utilToInt8(utilHexByteToUint8(hex.substring(6, 8)));
      const int8_t wz =
        utilToInt8(utilHexByteToUint8(hex.substring(8, 10)));

      if (SHOW_DRIVE) {
        Serial.print("DRIVE <- VX=");
        Serial.print(vx);
        Serial.print(" VY=");
        Serial.print(vy);
        Serial.print(" WZ=");
        Serial.println(wz);
      }

      chassisCtrlSetDriveCommand(vx, vy, wz);
      ++driveCommandCount;

      if (driveCommandCount % DRIVE_ACK_INTERVAL == 0) {
        String reply = "DRIVE OK PWR=";
        reply += String(chassisCtrlGetPowerPercent());
        im920CommSendText(reply);
      }
      break;
    }

    case ControlCommand::SET_WHEEL_GAIN: {
      if (hex.length() < 12) {
        return;
      }

      const uint8_t direction =
        utilHexByteToUint8(hex.substring(4, 6));
      const uint8_t wheelIndex =
        utilHexByteToUint8(hex.substring(6, 8));
      const float gain =
        static_cast<float>(parseUint16(hex, 8)) / GAIN_WIRE_SCALE;

      if (!chassisCtrlSetWheelGain(
            static_cast<ChassisGainDirection>(direction),
            wheelIndex,
            gain
          )) {
        Serial.println("WHEEL GAIN SET invalid parameters");
        return;
      }

      String reply = "WGS,";
      reply += String(direction);
      reply += ",";
      reply += String(wheelIndex);
      reply += ",";
      reply += String(gain, 3);
      im920CommSendText(reply);
      break;
    }

    case ControlCommand::GAIN_TUNE_START: {
      if (hex.length() < 12) {
        return;
      }

      const int8_t vx =
        utilToInt8(utilHexByteToUint8(hex.substring(4, 6)));
      const int8_t vy =
        utilToInt8(utilHexByteToUint8(hex.substring(6, 8)));
      const int8_t wz =
        utilToInt8(utilHexByteToUint8(hex.substring(8, 10)));
      const uint8_t durationUnits =
        utilHexByteToUint8(hex.substring(10, 12));

      if (durationUnits == 0) {
        Serial.println("GAIN TUNING invalid duration");
        return;
      }

      // 新しい試験では、前回結果の送信途中stateを必ず破棄する。
      resetGainTuningTxState();
      chassisCtrlStartGainTuning(
        vx,
        vy,
        wz,
        min(
          static_cast<uint32_t>(durationUnits) *
            GAIN_TUNING_DURATION_UNIT_MS,
          GAIN_TUNING_MAX_DURATION_MS
        )
      );
      im920CommSendText("TUNE START");
      break;
    }

    case ControlCommand::GAIN_TUNE_KEEPALIVE:
      break;

    case ControlCommand::GAIN_TUNE_RESULT_ACK: {
      if (hex.length() < 6) {
        return;
      }
      const uint8_t ackIndex =
        utilHexByteToUint8(hex.substring(4, 6));
      if (tuningTxState != GainTuningTxState::WAIT_REMOTE_ACK ||
          ackIndex != tuningTxIndex ||
          ackIndex > GAIN_TUNING_WHEEL_COUNT) {
        return;
      }

      Serial.print("[TUNE ACK] ");
      Serial.println(gainTuningResultLabel(ackIndex));

      if (ackIndex == GAIN_TUNING_WHEEL_COUNT) {
        chassisCtrlClearGainTuningResultReady();
        resetGainTuningTxState();
        return;
      }

      ++tuningTxIndex;
      tuningTxAttemptCount = 0;
      // PiからのACK直後に次のWGを送らず、無線の方向切替時間を確保する。
      tuningTxState = GainTuningTxState::TURNAROUND_GUARD;
      tuningTxAllowedMs =
        millis() + GAIN_TUNING_RESULT_TURNAROUND_GUARD_MS;
      break;
    }

    default:
      Serial.println("CONTROL unknown command");
      break;
  }
}

void sendGainTuningResultsIfReady() {
  if (tuningTxIndex < 0) {
    if (!chassisCtrlGainTuningResultReady()) {
      return;
    }
    tuningTxIndex = 0;
    tuningTxAttemptCount = 0;
    tuningTxState = GainTuningTxState::TURNAROUND_GUARD;
    tuningTxAllowedMs = millis();
  }

  if (tuningTxState == GainTuningTxState::WAIT_LOCAL_RESPONSE) {
    if (millis() - tuningTxStartedMs < GAIN_TUNING_TX_RESPONSE_TIMEOUT_MS) {
      return;
    }
    Serial.print("[TUNE TX] local response timeout ");
    Serial.println(gainTuningResultLabel(tuningTxIndex));
    scheduleGainTuningResultRetry();
    return;
  }

  if (tuningTxState == GainTuningTxState::WAIT_REMOTE_ACK) {
    if (millis() - tuningTxStartedMs < GAIN_TUNING_RESULT_ACK_TIMEOUT_MS) {
      return;
    }
    Serial.print("[TUNE ACK] timeout ");
    Serial.println(gainTuningResultLabel(tuningTxIndex));
    scheduleGainTuningResultRetry();
    return;
  }

  if (tuningTxState != GainTuningTxState::TURNAROUND_GUARD ||
      static_cast<int32_t>(millis() - tuningTxAllowedMs) < 0) {
    return;
  }

  if (tuningTxAttemptCount > GAIN_TUNING_RESULT_MAX_RETRIES) {
    failGainTuningResultTx();
    return;
  }

  String message;
  if (tuningTxIndex < GAIN_TUNING_WHEEL_COUNT) {
    const int wheelIndex = tuningTxIndex;
    const ChassisGainTuningResult result =
      chassisCtrlGetGainTuningResult(wheelIndex);
    message = "WG";
    message += String(wheelIndex);
    message += ",";
    message += String(static_cast<int>(lroundf(result.meanAbsoluteRpm)));
    message += ",";
    message += String(result.sampleCount);
    message += ",";
    message += String(static_cast<int>(lroundf(result.standardDeviationRpm)));
  } else if (tuningTxIndex == GAIN_TUNING_WHEEL_COUNT) {
    message = "WD";
  } else return;

  ++tuningTxAttemptCount;
  Serial.print("[TUNE TX] ");
  Serial.print(gainTuningResultLabel(tuningTxIndex));
  Serial.print(" attempt=");
  Serial.println(tuningTxAttemptCount);

  if (ENABLE_GAIN_TUNING_TX_LOG) {
    Serial.print("[TUNE TX] text=");
    Serial.println(message);
    Serial.print("[TUNE TX] text_len=");
    Serial.println(message.length());
    Serial.print("[TUNE TX] hex_len=");
    Serial.println(message.length() * 2);
    Serial.print("[TUNE TX] command_len=");
    Serial.println(5 + message.length() * 2);
    Serial.print("[TUNE TX] index=");
    Serial.println(tuningTxIndex);
  }
  im920CommSendText(message);
  tuningTxState = GainTuningTxState::WAIT_LOCAL_RESPONSE;
  tuningTxStartedMs = millis();
}

void handlePayloadHex(const String& hex) {
  if (hex.length() < 2) {
    return;
  }

  const uint8_t type =
    utilHexByteToUint8(hex.substring(0, 2));

  if (!isKnownPacketType(type)) {
    return;
  }

  lastRxMs = millis();
  pulseLed();

  if (type == static_cast<uint8_t>(PacketType::CONTROL)) {
    handleControlPacket(hex);
  } else if (type == static_cast<uint8_t>(PacketType::SERVO_SET)) {
    servoCtrlHandlePacket(hex);
  } else if (type == static_cast<uint8_t>(PacketType::STEPPER)) {
    stepperCtrlHandlePacket(hex);
  }
}

void handleIm920Line(const String& rawLine) {
  const String line = utilSanitizeAsciiLine(rawLine);

  if (line.length() == 0) {
    return;
  }

  if (SHOW_RAW) {
    Serial.print("IM920 RAW <- ");
    Serial.println(line);
  }

  if (line == "OK") {
    // TXDA後のOKはローカルIM920の処理結果であり、PiがWGを受信した
    // ことを保証しない。対応するapplication ACKまで次の結果へ進めない。
    if (tuningTxState == GainTuningTxState::WAIT_LOCAL_RESPONSE) {
      tuningTxState = GainTuningTxState::WAIT_REMOTE_ACK;
      tuningTxStartedMs = millis();
      Serial.print("[TUNE TX] local OK ");
      Serial.println(gainTuningResultLabel(tuningTxIndex));
    }
    return;
  }
  if (line == "NG") {
    if (tuningTxState == GainTuningTxState::WAIT_LOCAL_RESPONSE) {
      Serial.print("[TUNE TX] local NG ");
      Serial.println(gainTuningResultLabel(tuningTxIndex));
      scheduleGainTuningResultRetry();
    }
    return;
  }
  if (line.startsWith("IM920")) {
    return;
  }

  const String hex = extractPayloadHex(line);

  if (hex.length() > 0) {
    handlePayloadHex(hex);
  }
}

void readIm920Serial() {
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

    if (imLine.length() > 180) {
      imLine = "";
    }
  }
}

void readPcSerialCommand() {
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

    if (pcLine.length() > 120) {
      pcLine = "";
    }
  }
}
}

void im920CommBegin() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  IM920.begin(19200, SERIAL_8N1, IM920_RX, IM920_TX);
  delay(1000);

  printIm920Configuration();

  lastRxMs = millis();

  Serial.println("IM920 UART ready: RX=16 TX=17 19200bps");
}

void im920CommUpdate() {
  updateLed();
  readIm920Serial();
  readPcSerialCommand();
  sendGainTuningResultsIfReady();
}

void im920CommCheckTimeout() {
  if (millis() - lastRxMs <= COMM_TIMEOUT_MS) {
    return;
  }

  // result ACK待ちを含む通信状態もtimeoutを跨いで残さない。
  if (tuningTxState != GainTuningTxState::IDLE) {
    resetGainTuningTxState();
    chassisCtrlClearGainTuningResultReady();
  }

  if (chassisCtrlIsActive()) {
    Serial.println("COMM TIMEOUT -> CHASSIS STOP");
    chassisCtrlStop();
  }

  if (stepperCtrlIsRunning()) {
    Serial.println("COMM TIMEOUT -> STEPPER STOP");
    stepperCtrlStop();
  }

  lastRxMs = millis();
}

void im920CommSendText(const String& text) {
  if (!ENABLE_REPLY_TO_PI || text.length() == 0) {
    return;
  }
  if (text.length() > IM920_TXDA_MAX_PAYLOAD_BYTES) {
    Serial.print("IM920 TXDA payload too long: ");
    Serial.println(text.length());
    return;
  }

  const String command = "TXDA " + utilTextToHex(text);
  sendCommandToIm920(command);
}