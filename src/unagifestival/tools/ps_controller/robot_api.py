from typing import Protocol

from unagifestival.tools.ps_controller.models import (
    DriveCommand,
    ServoSetCommand,
)


class RobotTransport(Protocol):
    """意味コマンドをwire protocolへ送るtransport."""

    def send_stop(self) -> None: ...

    def send_emergency_stop(self) -> None: ...

    def send_change_power(self, delta: int) -> None: ...

    def send_drive(self, command: DriveCommand) -> None: ...

    def send_servo_set(self, command: ServoSetCommand) -> None: ...


class RobotApi:
    """Raspberry Piから利用するESP32の意味ベースAPI."""

    def __init__(self, transport: RobotTransport) -> None:
        self._transport = transport

    def stop(self) -> None:
        self._transport.send_stop()

    def emergency_stop(self) -> None:
        self._transport.send_emergency_stop()

    def change_power(self, delta: int) -> None:
        self._transport.send_change_power(delta)

    def drive(self, command: DriveCommand) -> None:
        self._transport.send_drive(command)

    def set_servo(self, command: ServoSetCommand) -> None:
        self._transport.send_servo_set(command)
