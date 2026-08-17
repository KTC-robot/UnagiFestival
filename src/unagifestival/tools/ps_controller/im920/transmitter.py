import logging

from unagifestival.tools.ps_controller.im920.model import (
    EncodedPacket,
    IM920Device,
)


class IM920Transmitter:
    """encode済みpacketをTXDA commandとしてIM920-HATへ送る."""

    def __init__(
        self,
        device: IM920Device,
        command_max_length: int,
        logger: logging.Logger,
    ) -> None:
        self._device = device
        self._command_max_length = command_max_length
        self._logger = logger

    def send(self, packet: EncodedPacket) -> None:
        """1つのencode済みpacketを送信する."""
        command = "TXDA " + packet.payload
        if len(command) > self._command_max_length:
            self._logger.warning("[ROBOT] SKIP command too long: %s", command)
            return
        self._logger.debug("[ROBOT] SEND %s -> %s", packet.label, command)
        try:
            self._device.write(command)
        except Exception:  # noqa: BLE001
            self._logger.warning(
                "[ROBOT] IM920 send failed: %s",
                command,
                exc_info=True,
            )
