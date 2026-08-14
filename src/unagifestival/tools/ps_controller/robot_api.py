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

    def send_set_wheel_gain(self, direction: int, wheel: int, gain: float) -> None: ...

    def send_gain_tune_start(
        self,
        command: DriveCommand,
        duration_ms: int,
    ) -> None: ...

    def send_gain_tune_keepalive(self) -> None: ...

    def send_gain_tune_result_ack(self, result_index: int) -> None: ...

    def send_step_assist_reset(self) -> None: ...

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

    def set_wheel_gain(self, direction: int, wheel: int, gain: float) -> None:
        self._transport.send_set_wheel_gain(direction, wheel, gain)

    def start_gain_tuning(
        self,
        command: DriveCommand,
        duration_ms: int,
    ) -> None:
        self._transport.send_gain_tune_start(command, duration_ms)

    def gain_tuning_keepalive(self) -> None:
        self._transport.send_gain_tune_keepalive()

    def ack_gain_tuning_result(self, result_index: int) -> None:
        """受信済みgain tuning結果のapplication ACKを送信する.

        0-3はWG0-WG3、4はWDを表す。duplicate結果にも毎回送信する.
        """
        self._transport.send_gain_tune_result_ack(result_index)

    def reset_step_assist(self) -> None:
        """ESP32の段差制御状態をNORMALへリセットする."""
        self._transport.send_step_assist_reset()

    def set_servo(self, command: ServoSetCommand) -> None:
        self._transport.send_servo_set(command)
