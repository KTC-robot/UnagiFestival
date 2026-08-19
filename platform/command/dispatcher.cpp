#include "command/dispatcher.hpp"

#include "chassis_ctrl/chassis_ctrl.hpp"
#include "step_assist/step_assist_ctrl.hpp"

CommandDispatchResult dispatchCommand(const Command& command) {
  // Decoderがwire形式を意味的なCommandへ変換済みなので、ここではpacketを
  // 再解析せず、種類に対応するSystem APIへ値を引き渡すだけにする。
  CommandDispatchResult result;

  switch (command.type) {
    case CommandType::STOP:
      chassisCtrlStop();
      result.executed = true;
      result.resetGainTuningTx = true;
      result.reply = CommandReply::CTRL_STOP;
      break;
    case CommandType::EMERGENCY_STOP:
      chassisCtrlStop();
      result.executed = true;
      result.resetGainTuningTx = true;
      result.reply = CommandReply::CTRL_ESTOP;
      break;
    case CommandType::CHANGE_POWER:
      chassisCtrlChangePower(command.powerDelta);
      result.executed = true;
      result.reply = CommandReply::POWER;
      break;
    case CommandType::DRIVE:
      chassisCtrlSetDriveCommand(
        command.drive.vx, command.drive.vy, command.drive.wz
      );
      result.executed = true;
      result.driveExecuted = true;
      break;
    case CommandType::SET_WHEEL_GAIN:
      result.executed = chassisCtrlSetWheelGain(
        static_cast<ChassisGainDirection>(command.wheelGain.direction),
        command.wheelGain.wheelIndex,
        command.wheelGain.gain
      );
      if (result.executed) {
        result.reply = CommandReply::WHEEL_GAIN;
        result.wheelGainDirection = command.wheelGain.direction;
        result.wheelGainIndex = command.wheelGain.wheelIndex;
        result.wheelGain = command.wheelGain.gain;
      }
      break;
    case CommandType::GAIN_TUNE_START:
      chassisCtrlStartGainTuning(
        command.gainTuneStart.vx,
        command.gainTuneStart.vy,
        command.gainTuneStart.wz,
        command.gainTuneStart.durationMs
      );
      result.executed = true;
      result.resetGainTuningTx = true;
      result.reply = CommandReply::TUNE_START;
      break;
    case CommandType::GAIN_TUNE_KEEPALIVE:
      result.executed = true;
      break;
    case CommandType::GAIN_TUNE_RESULT_ACK:
      result.executed = true;
      result.gainTuningResultAck = true;
      result.gainTuningResultIndex = command.gainTuneResultIndex;
      break;
    case CommandType::STEP_ASSIST_RESET:
      stepAssistCtrlReset();
      result.executed = true;
      result.reply = CommandReply::STEP_RESET;
      break;
  }

  return result;
}
