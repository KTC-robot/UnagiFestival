from enum import IntEnum


class ControlCommand(IntEnum):
    """ESP32 Control APIのwire command ID."""

    STOP = 0x01
    EMERGENCY_STOP = 0x02
    CHANGE_POWER = 0x03
    DRIVE = 0x04
    SET_WHEEL_GAIN = 0x05
    GAIN_TUNE_START = 0x06
    GAIN_TUNE_KEEPALIVE = 0x07
    GAIN_TUNE_RESULT_ACK = 0x08
    STEP_ASSIST_RESET = 0x09


class PacketType(IntEnum):
    """Raspberry PiからESP32へ送るwire packet種別."""

    CONTROL = 0x43
    SERVO_SET = 0x53
    SERVO_SET_ALL = 0x54
