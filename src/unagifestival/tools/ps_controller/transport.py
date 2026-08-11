import logging

from collections.abc import Callable
from enum import IntEnum
from typing import Protocol

from unagifestival.tools.ps_controller.robot_api import ControlCommand


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

        self._logger.info("[ROBOT] SEND %s -> %s", label, command)

        try:
            self._im920.Write_920(command)
            self._on_transmit()
        except Exception:  # noqa: BLE001
            self._logger.warning(
                "[ROBOT] IM920 send failed: %s",
                command,
                exc_info=True,
            )

    def send_control(self, command: ControlCommand, *parameters: int) -> None:
        payload = self._byte_to_hex(PacketType.CONTROL)
        payload += self._byte_to_hex(command)
        payload += "".join(self._byte_to_hex(value) for value in parameters)
        self._send_payload(payload, f"CONTROL {command.name}")

    def send_servo_set(self, channel: int, angle: int) -> None:
        payload = self._byte_to_hex(PacketType.SERVO_SET)
        payload += self._byte_to_hex(channel)
        payload += self._byte_to_hex(angle)
        self._send_payload(payload, "SERVO_SET")

    def read(self) -> str:
        return self._im920.Read_920()

    def cleanup(self) -> None:
        self._im920.gpio_clean()
