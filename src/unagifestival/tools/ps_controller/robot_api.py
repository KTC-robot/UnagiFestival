from enum import IntEnum
from typing import Protocol


class ControlCommand(IntEnum):
    """ESP32 Control APIのコマンドID."""

    STOP = 0x01
    EMERGENCY_STOP = 0x02
    CHANGE_POWER = 0x03
    DRIVE = 0x04


class RobotTransport(Protocol):
    """意味コマンドをwire protocolへ送るtransport."""

    def send_control(self, command: ControlCommand, *parameters: int) -> None: ...

    def send_servo_set(self, channel: int, angle: int) -> None: ...


class RobotApi:
    """Raspberry Piから利用するESP32の意味ベースAPI."""

    def __init__(self, transport: RobotTransport) -> None:
        self._transport = transport

    def stop(self) -> None:
        self._transport.send_control(ControlCommand.STOP)

    def emergency_stop(self) -> None:
        self._transport.send_control(ControlCommand.EMERGENCY_STOP)

    def change_power(self, delta: int) -> None:
        self._transport.send_control(ControlCommand.CHANGE_POWER, delta)

    def drive(self, vx: int, vy: int, wz: int) -> None:
        self._transport.send_control(ControlCommand.DRIVE, vx, vy, wz)

    def set_servo(self, channel: int, angle: int) -> None:
        self._transport.send_servo_set(channel, angle)
