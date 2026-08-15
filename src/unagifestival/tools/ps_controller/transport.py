import logging

from collections import deque
from collections.abc import Callable
from enum import IntEnum
from typing import Protocol

from unagifestival.tools.ps_controller.models import (
    DriveCommand,
    ServoSetCommand,
)


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


GAIN_WIRE_SCALE = 1000
GAIN_TUNING_DURATION_UNIT_MS = 100
GAIN_TUNING_MAX_DURATION_MS = 10_000
GAIN_TUNING_DONE_ACK_INDEX = 4
WHEEL_INDEX_MIN = 0
WHEEL_INDEX_MAX = 3
GAIN_DIRECTION_MIN = 0
GAIN_DIRECTION_MAX = 3
WHEEL_GAIN_MIN = 0.50
WHEEL_GAIN_MAX = 1.50
UINT16_MAX_VALUE = 0xFFFF
INVALID_WHEEL_MESSAGE = "wheel must be between 0 and 3"
INVALID_DIRECTION_MESSAGE = "direction must be between 0 and 3"
INVALID_GAIN_MESSAGE = "gain must be between 0.50 and 1.50"
INVALID_TUNING_DURATION_MESSAGE = "duration_ms must be 100..10000 in 100 ms units"
INVALID_RESULT_ACK_MESSAGE = "result_index must be between 0 and 4"
INVALID_SERVO_ANGLE_MESSAGE = "angle must be between 0 and 180"
SERVO_ANGLE_MAX = 180


class Im920Device(Protocol):
    """IM920-HATドライバーでtransportが利用する操作."""

    def Write_920(self, command: str) -> None: ...  # noqa: N802

    def Read_920(self) -> str: ...  # noqa: N802

    def gpio_clean(self) -> None: ...


class PacketType(IntEnum):
    """Raspberry PiからESP32へ送るwire packet種別."""

    CONTROL = 0x43
    SERVO_SET = 0x53
    SERVO_SET_ALL = 0x54


class Im920Transport:
    """意味コマンドをIM920のTXDA wire protocolへ変換する."""

    def __init__(
        self,
        im920: Im920Device,
        command_max_length: int,
        on_transmit: Callable[[], None],
        logger: logging.Logger,
    ) -> None:
        self._im920 = im920
        self._command_max_length = command_max_length
        self._on_transmit = on_transmit
        self._logger = logger
        self._pending_reads: deque[str] = deque()

    @staticmethod
    def _byte_to_hex(value: int) -> str:
        return f"{value & 0xFF:02X}"

    def _send_payload(self, payload: str, label: str) -> None:
        command = "TXDA " + payload

        if len(command) > self._command_max_length:
            self._logger.warning("[ROBOT] SKIP command too long: %s", command)
            return

        self._logger.debug("[ROBOT] SEND %s -> %s", label, command)

        try:
            self._im920.Write_920(command)
            self._on_transmit()
        except Exception:  # noqa: BLE001
            self._logger.warning(
                "[ROBOT] IM920 send failed: %s",
                command,
                exc_info=True,
            )

    def _send_control(self, command: ControlCommand, parameters: str = "") -> None:
        payload = self._byte_to_hex(PacketType.CONTROL)
        payload += self._byte_to_hex(command)
        payload += parameters
        self._send_payload(payload, f"CONTROL {command.name}")

    def send_stop(self) -> None:
        self._send_control(ControlCommand.STOP)

    def send_emergency_stop(self) -> None:
        self._send_control(ControlCommand.EMERGENCY_STOP)

    def send_change_power(self, delta: int) -> None:
        self._send_control(ControlCommand.CHANGE_POWER, self._byte_to_hex(delta))

    def send_drive(self, command: DriveCommand) -> None:
        parameters = self._byte_to_hex(command.vx)
        parameters += self._byte_to_hex(command.vy)
        parameters += self._byte_to_hex(command.wz)
        self._send_control(ControlCommand.DRIVE, parameters)

    def send_set_wheel_gain(self, direction: int, wheel: int, gain: float) -> None:
        if not GAIN_DIRECTION_MIN <= direction <= GAIN_DIRECTION_MAX:
            raise ValueError(INVALID_DIRECTION_MESSAGE)
        if not WHEEL_INDEX_MIN <= wheel <= WHEEL_INDEX_MAX:
            raise ValueError(INVALID_WHEEL_MESSAGE)
        if not WHEEL_GAIN_MIN <= gain <= WHEEL_GAIN_MAX:
            raise ValueError(INVALID_GAIN_MESSAGE)

        gain_scaled = round(gain * GAIN_WIRE_SCALE)
        if not 0 <= gain_scaled <= UINT16_MAX_VALUE:
            raise ValueError(INVALID_GAIN_MESSAGE)
        parameters = self._byte_to_hex(direction)
        parameters += self._byte_to_hex(wheel)
        parameters += f"{gain_scaled:04X}"
        self._send_control(ControlCommand.SET_WHEEL_GAIN, parameters)

    def send_gain_tune_start(
        self,
        command: DriveCommand,
        duration_ms: int,
    ) -> None:
        if (
            duration_ms < GAIN_TUNING_DURATION_UNIT_MS
            or duration_ms > GAIN_TUNING_MAX_DURATION_MS
            or duration_ms % GAIN_TUNING_DURATION_UNIT_MS != 0
        ):
            raise ValueError(INVALID_TUNING_DURATION_MESSAGE)

        parameters = self._byte_to_hex(command.vx)
        parameters += self._byte_to_hex(command.vy)
        parameters += self._byte_to_hex(command.wz)
        parameters += self._byte_to_hex(duration_ms // GAIN_TUNING_DURATION_UNIT_MS)
        self._send_control(ControlCommand.GAIN_TUNE_START, parameters)

    def send_gain_tune_keepalive(self) -> None:
        self._send_control(ControlCommand.GAIN_TUNE_KEEPALIVE)

    def send_gain_tune_result_ack(self, result_index: int) -> None:
        """受信したWG0-WG3またはWDをESP32へ通知する.

        Args:
            result_index: 0-3はWG0-WG3、4はWD.

        Raises:
            ValueError: result_indexが0-4の範囲外の場合.
        """
        if not 0 <= result_index <= GAIN_TUNING_DONE_ACK_INDEX:
            raise ValueError(INVALID_RESULT_ACK_MESSAGE)
        self._send_control(
            ControlCommand.GAIN_TUNE_RESULT_ACK,
            self._byte_to_hex(result_index),
        )

    def send_step_assist_reset(self) -> None:
        """ESP32の段差制御状態をNORMALへ戻す."""
        self._send_control(ControlCommand.STEP_ASSIST_RESET)

    def send_servo_set(self, command: ServoSetCommand) -> None:
        payload = self._byte_to_hex(PacketType.SERVO_SET)
        payload += self._byte_to_hex(command.channel)
        payload += self._byte_to_hex(command.angle)

        self._logger.info(
            "[SERVO][TX] CH=%d ANGLE=%d PAYLOAD=%s COMMAND=TXDA %s",
            command.channel,
            command.angle,
            payload,
            payload,
        )

        self._send_payload(payload, "SERVO_SET")

    def send_servo_set_all(self, angle: int) -> None:
        """全論理サーボへ同じ角度を設定するpacketを送信する.

        Args:
            angle: 設定角度。0〜180度.

        Raises:
            ValueError: angleが0〜180度の範囲外の場合.
        """
        if not 0 <= angle <= SERVO_ANGLE_MAX:
            raise ValueError(INVALID_SERVO_ANGLE_MESSAGE)

        payload = self._byte_to_hex(PacketType.SERVO_SET_ALL)
        payload += self._byte_to_hex(angle)
        self._logger.info(
            "[SERVO][TX ALL] ANGLE=%d PAYLOAD=%s",
            angle,
            payload,
        )
        self._send_payload(payload, "SERVO_SET_ALL")

    def read(self) -> str:
        if self._pending_reads:
            return self._pending_reads.popleft()

        raw = self._im920.Read_920()
        if not raw:
            return ""

        frames = [frame for frame in raw.splitlines() if frame.strip()]
        if not frames:
            return ""
        self._pending_reads.extend(frames[1:])
        return frames[0]

    def cleanup(self) -> None:
        self._im920.gpio_clean()
