#include "command/decoder.hpp"

#include <algorithm>

#include "command/constants.hpp"

using namespace CommandProtocol;

namespace {
// payloadは「2文字で1byte」を表す16進ASCII文字列である。
bool hexNibble(char value, uint8_t& nibble) {
  if (value >= '0' && value <= '9') {
    nibble = static_cast<uint8_t>(value - '0');
    return true;
  }
  if (value >= 'A' && value <= 'F') {
    nibble = static_cast<uint8_t>(value - 'A' + 10);
    return true;
  }
  if (value >= 'a' && value <= 'f') {
    nibble = static_cast<uint8_t>(value - 'a' + 10);
    return true;
  }
  return false;
}

bool byteAt(std::string_view payload, size_t offset, uint8_t& value) {
  // offsetは文字位置であり、1byte進める場合は2文字進める。
  if (offset + 2 > payload.size()) return false;
  uint8_t high = 0;
  uint8_t low = 0;
  if (!hexNibble(payload[offset], high) ||
      !hexNibble(payload[offset + 1], low)) {
    return false;
  }
  value = static_cast<uint8_t>((high << 4) | low);
  return true;
}

bool uint16At(std::string_view payload, size_t offset, uint16_t& value) {
  uint8_t high = 0;
  uint8_t low = 0;
  if (!byteAt(payload, offset, high) || !byteAt(payload, offset + 2, low)) {
    return false;
  }
  // wire上のbig endian順から16bit値を復元する。
  value = static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
  return true;
}

int8_t toInt8(uint8_t value) {
  // 0x80〜0xFFを負数として扱い、Python側の符号付き指令を復元する。
  return value < 0x80 ? static_cast<int8_t>(value)
                      : static_cast<int8_t>(static_cast<int16_t>(value) - 256);
}

bool decodeControl(std::string_view payload, Command& command) {
  // 先頭byteはPacketType、その次のbyteがCONTROL内のCommand IDである。
  uint8_t commandId = 0;
  if (!byteAt(payload, 2, commandId)) return false;

  uint8_t first = 0;
  uint8_t second = 0;
  uint8_t third = 0;
  switch (static_cast<ControlCommand>(commandId)) {
    case ControlCommand::STOP:
      command.type = CommandType::STOP;
      return true;
    case ControlCommand::EMERGENCY_STOP:
      command.type = CommandType::EMERGENCY_STOP;
      return true;
    case ControlCommand::DRIVE:
      if (!byteAt(payload, 4, first) || !byteAt(payload, 6, second) ||
          !byteAt(payload, 8, third)) return false;
      command.type = CommandType::DRIVE;
      command.drive = {toInt8(first), toInt8(second), toInt8(third)};
      return true;
    case ControlCommand::SET_WHEEL_GAIN: {
      uint16_t scaledGain = 0;
      if (!byteAt(payload, 4, first) || !byteAt(payload, 6, second) ||
          !uint16At(payload, 8, scaledGain)) return false;
      // 小数gainはwire上では1000倍したuint16として送られる。
      const float gain = static_cast<float>(scaledGain) / GAIN_WIRE_SCALE;
      if (first >= GAIN_TUNING_WHEEL_COUNT ||
          second >= GAIN_TUNING_WHEEL_COUNT || gain < 0.5f || gain > 1.5f) {
        return false;
      }
      command.type = CommandType::SET_WHEEL_GAIN;
      command.wheelGain = {first, second, gain};
      return true;
    }
    case ControlCommand::GAIN_TUNE_START:
      if (!byteAt(payload, 4, first) || !byteAt(payload, 6, second) ||
          !byteAt(payload, 8, third)) return false;
      {
        uint8_t durationUnits = 0;
        if (!byteAt(payload, 10, durationUnits) || durationUnits == 0) {
          return false;
        }
        command.type = CommandType::GAIN_TUNE_START;
        // 計測時間は100ms単位の1byteで受け取り、安全上限以内へ制限する。
        command.gainTuneStart = {
          toInt8(first), toInt8(second), toInt8(third),
          std::min(
            static_cast<uint32_t>(durationUnits) * GAIN_TUNING_DURATION_UNIT_MS,
            GAIN_TUNING_MAX_DURATION_MS
          )
        };
      }
      return true;
    case ControlCommand::GAIN_TUNE_KEEPALIVE:
      command.type = CommandType::GAIN_TUNE_KEEPALIVE;
      return true;
    case ControlCommand::GAIN_TUNE_RESULT_ACK:
      if (!byteAt(payload, 4, first) || first > GAIN_TUNING_WHEEL_COUNT) {
        return false;
      }
      command.type = CommandType::GAIN_TUNE_RESULT_ACK;
      command.gainTuneResultIndex = first;
      return true;
    case ControlCommand::STEP_ASSIST_RESET:
      command.type = CommandType::STEP_ASSIST_RESET;
      return true;
    case ControlCommand::AIR_FIRE_START:
      command.type = CommandType::AIR_FIRE_START;
      return true;
    case ControlCommand::AIR_FIRE_STOP:
      command.type = CommandType::AIR_FIRE_STOP;
      return true;
    case ControlCommand::MD20A_SET_STATE:
      if (!byteAt(payload, 4, first) || first > 2) return false;
      command.type = CommandType::MD20A_SET_STATE;
      command.md20aState = first;
      return true;
    default:
      return false;
  }
}
}  // namespace

bool decodeCommand(std::string_view payloadHex, Command& command) {
  uint8_t packetType = 0;
  if (!byteAt(payloadHex, 0, packetType)) return false;

  switch (static_cast<PacketType>(packetType)) {
    case PacketType::CONTROL:
      return decodeControl(payloadHex, command);
    default:
      return false;
  }
}
