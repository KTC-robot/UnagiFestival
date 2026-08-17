import logging

from unagifestival.tools.ps_controller.im920.model import (
    EncodedPacket,
    IM920Device,
)


class IM920Transmitter:
    """
    Properties:
        なし。

    About:
        encode済みpacketをTXDA commandへ整形し、IM920-HATへ送信する。
    """

    def __init__(
        self,
        device: IM920Device,
        command_max_length: int,
        logger: logging.Logger,
    ) -> None:
        """
        Args:
            device: TXDA commandを書き込むIM920 device。
            command_max_length: command文字列の最大長。
            logger: 送信状況を記録するlogger。

        Returns:
            なし。

        About:
            送信先device、長さ制限、loggerを保持する。
        """
        self._device = device
        self._command_max_length = command_max_length
        self._logger = logger

    def send(self, packet: EncodedPacket) -> None:
        """
        Args:
            packet: 送信するencode済みpacket。

        Returns:
            なし。

        About:
            packetをTXDA commandへ変換し、長さを検証してdeviceへ書き込む。
        """
        command = "TXDA " + packet.payload
        if len(command) > self._command_max_length:
            self._logger.warning("[ROBOT] commandが長すぎるため送信しません: %s", command)
            return
        self._logger.debug("[ROBOT] packetを送信します: label=%s command=%s", packet.label, command)
        try:
            self._device.write(command)
        except Exception:  # noqa: BLE001
            self._logger.warning(
                "[ROBOT] IM920への送信に失敗しました: %s",
                command,
                exc_info=True,
            )
