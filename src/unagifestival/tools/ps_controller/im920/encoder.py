from unagifestival.tools.ps_controller.im920.constants import (
    GAIN_DIRECTION_MAX,
    GAIN_DIRECTION_MIN,
    GAIN_TUNING_DONE_ACK_INDEX,
    GAIN_TUNING_DURATION_UNIT_MS,
    GAIN_TUNING_MAX_DURATION_MS,
    GAIN_WIRE_SCALE,
    INVALID_DIRECTION_MESSAGE,
    INVALID_GAIN_MESSAGE,
    INVALID_RESULT_ACK_MESSAGE,
    INVALID_SERVO_ANGLE_MESSAGE,
    INVALID_TUNING_DURATION_MESSAGE,
    INVALID_WHEEL_MESSAGE,
    SERVO_ANGLE_MAX,
    UINT16_MAX_VALUE,
    WHEEL_GAIN_MAX,
    WHEEL_GAIN_MIN,
    WHEEL_INDEX_MAX,
    WHEEL_INDEX_MIN,
)
from unagifestival.tools.ps_controller.im920.enum import (
    ControlCommand,
    PacketType,
)
from unagifestival.tools.ps_controller.im920.model import (
    ChangePowerCommand,
    DriveCommand,
    EmergencyStopCommand,
    EncodedPacket,
    GainTuneKeepaliveCommand,
    GainTuneResultAckCommand,
    GainTuneStartCommand,
    IM920Command,
    SetWheelGainCommand,
    StepAssistResetCommand,
    StopCommand,
)
from unagifestival.tools.ps_controller.servo.model import (
    ServoSetAllCommand,
    ServoSetCommand,
)


def _byte_to_hex(value: int) -> str:
    """
    Args:
        value: 1byteとして変換する整数値。
    Returns:
        下位8bitを2桁の16進数で表した文字列。
    About:
        wire packetへ格納する数値を固定長の16進数へ変換する。
    """
    return f"{value & 0xFF:02X}"


def _control(command: ControlCommand, parameters: str = "") -> EncodedPacket:
    """
    Args:
        command: CONTROL packetへ格納するCommand ID。
        parameters: Command IDに続ける16進数parameter。
    Returns:
        CONTROL種別としてencodeしたpacket。
    About:
        packet種別、Command ID、parameterを送信payloadへ結合する。
    """
    payload = _byte_to_hex(PacketType.CONTROL)
    payload += _byte_to_hex(command)
    return EncodedPacket(payload + parameters, f"CONTROL {command.name}")


def encode_command(  # noqa: C901, PLR0911, PLR0912
    command: IM920Command,
) -> EncodedPacket:
    """
    Args:
        command: encode対象の意味Command。
    Returns:
        IM920のTXDAで送信できる16進数payloadとlabel。
    About:
        Commandの型に応じて値を検証し、対応するwire packetへ変換する。
    """
    if isinstance(command, StopCommand):
        return _control(ControlCommand.STOP)
    if isinstance(command, EmergencyStopCommand):
        return _control(ControlCommand.EMERGENCY_STOP)
    if isinstance(command, ChangePowerCommand):
        return _control(ControlCommand.CHANGE_POWER, _byte_to_hex(command.delta))
    if isinstance(command, DriveCommand):
        parameters = _byte_to_hex(command.vx)
        parameters += _byte_to_hex(command.vy)
        parameters += _byte_to_hex(command.wz)
        return _control(ControlCommand.DRIVE, parameters)
    if isinstance(command, SetWheelGainCommand):
        if not GAIN_DIRECTION_MIN <= command.direction <= GAIN_DIRECTION_MAX:
            raise ValueError(INVALID_DIRECTION_MESSAGE)
        if not WHEEL_INDEX_MIN <= command.wheel <= WHEEL_INDEX_MAX:
            raise ValueError(INVALID_WHEEL_MESSAGE)
        if not WHEEL_GAIN_MIN <= command.gain <= WHEEL_GAIN_MAX:
            raise ValueError(INVALID_GAIN_MESSAGE)
        gain_scaled = round(command.gain * GAIN_WIRE_SCALE)
        if not 0 <= gain_scaled <= UINT16_MAX_VALUE:
            raise ValueError(INVALID_GAIN_MESSAGE)
        parameters = _byte_to_hex(command.direction)
        parameters += _byte_to_hex(command.wheel)
        parameters += f"{gain_scaled:04X}"
        return _control(ControlCommand.SET_WHEEL_GAIN, parameters)
    if isinstance(command, GainTuneStartCommand):
        duration_ms = command.duration_ms
        if (
            duration_ms < GAIN_TUNING_DURATION_UNIT_MS
            or duration_ms > GAIN_TUNING_MAX_DURATION_MS
            or duration_ms % GAIN_TUNING_DURATION_UNIT_MS != 0
        ):
            raise ValueError(INVALID_TUNING_DURATION_MESSAGE)
        drive = command.drive
        parameters = _byte_to_hex(drive.vx)
        parameters += _byte_to_hex(drive.vy)
        parameters += _byte_to_hex(drive.wz)
        parameters += _byte_to_hex(duration_ms // GAIN_TUNING_DURATION_UNIT_MS)
        return _control(ControlCommand.GAIN_TUNE_START, parameters)
    if isinstance(command, GainTuneKeepaliveCommand):
        return _control(ControlCommand.GAIN_TUNE_KEEPALIVE)
    if isinstance(command, GainTuneResultAckCommand):
        if not 0 <= command.result_index <= GAIN_TUNING_DONE_ACK_INDEX:
            raise ValueError(INVALID_RESULT_ACK_MESSAGE)
        return _control(
            ControlCommand.GAIN_TUNE_RESULT_ACK,
            _byte_to_hex(command.result_index),
        )
    if isinstance(command, StepAssistResetCommand):
        return _control(ControlCommand.STEP_ASSIST_RESET)
    if isinstance(command, ServoSetCommand):
        payload = _byte_to_hex(PacketType.SERVO_SET)
        payload += _byte_to_hex(command.channel)
        payload += _byte_to_hex(command.angle)
        return EncodedPacket(payload, "SERVO_SET")
    if isinstance(command, ServoSetAllCommand):
        if not 0 <= command.angle <= SERVO_ANGLE_MAX:
            raise ValueError(INVALID_SERVO_ANGLE_MESSAGE)
        payload = _byte_to_hex(PacketType.SERVO_SET_ALL)
        payload += _byte_to_hex(command.angle)
        return EncodedPacket(payload, "SERVO_SET_ALL")
    raise TypeError(type(command).__name__)
