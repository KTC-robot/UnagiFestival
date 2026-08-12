import logging

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
    SET_GAIN = 0x05
    GAIN_TUNE_START = 0x06
    GAIN_TUNE_KEEPALIVE = 0x07


GAIN_WIRE_SCALE = 1000
GAIN_TUNING_DURATION_UNIT_MS = 100
GAIN_TUNING_MAX_DURATION_MS = 10_000
MOTOR_ID_MIN = 1
MOTOR_ID_MAX = 4
UINT16_MAX_VALUE = 0xFFFF
INVALID_MOTOR_ID_MESSAGE = "motor_id must be between 1 and 4"
INVALID_GAIN_MESSAGE = "scaled gain must fit in uint16"
INVALID_TUNING_DURATION_MESSAGE = (
    "duration_ms must be 100..10000 in 100 ms units"
)


class Im920Device(Protocol):
    """IM920-HATドライバーでtransportが利用する操作."""

    def Write_920(self, command: str) -> None: ...  # noqa: N802

    def Read_920(self) -> str: ...  # noqa: N802

    def gpio_clean(self) -> None: ...


class PacketType(IntEnum):
    """Raspberry PiからESP32へ送るwire packet種別."""

    CONTROL = 0x43
    SERVO_SET = 0x53


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

    def send_set_gain(self, motor_id: int, kp: float, ki: float) -> None:
        if not MOTOR_ID_MIN <= motor_id <= MOTOR_ID_MAX:
            raise ValueError(INVALID_MOTOR_ID_MESSAGE)

        kp_scaled = round(kp * GAIN_WIRE_SCALE)
        ki_scaled = round(ki * GAIN_WIRE_SCALE)
        if (
            not 0 <= kp_scaled <= UINT16_MAX_VALUE
            or not 0 <= ki_scaled <= UINT16_MAX_VALUE
        ):
            raise ValueError(INVALID_GAIN_MESSAGE)

        parameters = self._byte_to_hex(motor_id)
        parameters += f"{kp_scaled:04X}{ki_scaled:04X}"
        self._send_control(ControlCommand.SET_GAIN, parameters)

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
        parameters += self._byte_to_hex(
            duration_ms // GAIN_TUNING_DURATION_UNIT_MS
        )
        self._send_control(ControlCommand.GAIN_TUNE_START, parameters)

    def send_gain_tune_keepalive(self) -> None:
        self._send_control(ControlCommand.GAIN_TUNE_KEEPALIVE)

    def send_servo_set(self, command: ServoSetCommand) -> None:
        payload = self._byte_to_hex(PacketType.SERVO_SET)
        payload += self._byte_to_hex(command.channel)
        payload += self._byte_to_hex(command.angle)
        self._send_payload(payload, "SERVO_SET")

    def read(self) -> str:
        return self._im920.Read_920()

    def cleanup(self) -> None:
        self._im920.gpio_clean()
