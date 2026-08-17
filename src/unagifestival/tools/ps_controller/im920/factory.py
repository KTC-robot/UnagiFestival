from unagifestival.tools.ps_controller.im920.model import (
    ChangePowerCommand,
    DriveCommand,
    EmergencyStopCommand,
    GainTuneKeepaliveCommand,
    GainTuneResultAckCommand,
    GainTuneStartCommand,
    SetWheelGainCommand,
    StepAssistResetCommand,
    StopCommand,
)
from unagifestival.tools.ps_controller.servo.model import (
    ServoSetAllCommand,
    ServoSetCommand,
)


class CommandFactory:
    """Handler等の意味操作からwire非依存Commandを生成する."""

    @staticmethod
    def drive(vx: int, vy: int, wz: int) -> DriveCommand:
        return DriveCommand(vx, vy, wz)

    @staticmethod
    def stop() -> StopCommand:
        return StopCommand()

    @staticmethod
    def emergency_stop() -> EmergencyStopCommand:
        return EmergencyStopCommand()

    @staticmethod
    def change_power(delta: int) -> ChangePowerCommand:
        return ChangePowerCommand(delta)

    @staticmethod
    def reset_step_assist() -> StepAssistResetCommand:
        return StepAssistResetCommand()

    @staticmethod
    def servo_set(channel: int, angle: int) -> ServoSetCommand:
        return ServoSetCommand(channel, angle)

    @staticmethod
    def servo_set_all(angle: int) -> ServoSetAllCommand:
        return ServoSetAllCommand(angle)

    @staticmethod
    def set_wheel_gain(
        direction: int,
        wheel: int,
        gain: float,
    ) -> SetWheelGainCommand:
        return SetWheelGainCommand(direction, wheel, gain)

    @staticmethod
    def start_gain_tuning(
        drive: DriveCommand,
        duration_ms: int,
    ) -> GainTuneStartCommand:
        return GainTuneStartCommand(drive, duration_ms)

    @staticmethod
    def gain_tuning_keepalive() -> GainTuneKeepaliveCommand:
        return GainTuneKeepaliveCommand()

    @staticmethod
    def ack_gain_tuning_result(result_index: int) -> GainTuneResultAckCommand:
        return GainTuneResultAckCommand(result_index)
