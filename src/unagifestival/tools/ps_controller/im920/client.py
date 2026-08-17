import logging

from typing import Protocol

from unagifestival.tools.ps_controller.im920.constants import (
    IM920_COMMAND_MAX_LENGTH,
    SLAVE_ADDRESS,
)
from unagifestival.tools.ps_controller.im920.decoder import decode_frame
from unagifestival.tools.ps_controller.im920.encoder import encode_command
from unagifestival.tools.ps_controller.im920.model import (
    IM920Command,
    IM920Device,
    IM920Response,
)
from unagifestival.tools.ps_controller.im920.receiver import IM920Receiver
from unagifestival.tools.ps_controller.im920.transmitter import IM920Transmitter


class IM920ClientProtocol(Protocol):
    """
    Properties:
        なし。

    About:
        Handlerが利用するIM920送受信Facadeのinterfaceを定義する。
    """

    def send(self, command: IM920Command) -> None:
        """
        Args:
            command: ESP32へ送信する意味Command。

        Returns:
            なし。

        About:
            意味CommandをIM920経由で送信する。
        """
        ...

    def poll(self) -> IM920Response | None:
        """
        Args:
            なし。

        Returns:
            受信データがある場合は復号済みResponse、ない場合はNone。

        About:
            IM920からapplication responseを最大1件取得する。
        """
        ...

    def close(self) -> None:
        """
        Args:
            なし。

        Returns:
            なし。

        About:
            IM920通信で所有するhardware resourceを解放する。
        """
        ...


class IM920Client:
    """
    Properties:
        なし。

    About:
        Encoder、Transmitter、Receiver、Decoderを束ね、意味Commandの送信と
        application responseの受信だけを外部へ公開するFacade。
    """

    def __init__(
        self,
        device: IM920Device,
        *,
        command_max_length: int = IM920_COMMAND_MAX_LENGTH,
        logger: logging.Logger | None = None,
    ) -> None:
        """
        Args:
            device: IM920-HATの低レイヤ入出力を提供するdevice。
            command_max_length: 送信可能なcommand文字列の最大長。
            logger: 通信ログの出力先。

        Returns:
            なし。

        About:
            送受信処理を構成し、利用するdeviceとloggerを保持する。
        """
        self._device = device
        self._logger = logger or logging.getLogger("unagi_log")
        self._transmitter = IM920Transmitter(
            device,
            command_max_length,
            self._logger,
        )
        self._receiver = IM920Receiver(device)

    def send(self, command: IM920Command) -> None:
        """
        Args:
            command: ESP32へ送信する意味Command。

        Returns:
            なし。

        About:
            意味Commandをpacketへencodeし、TransmitterからIM920-HATへ送る。
        """
        self._transmitter.send(encode_command(command))

    def poll(self) -> IM920Response | None:
        """
        Args:
            なし。

        Returns:
            正常なframeを受信した場合は復号済みResponse、データがない場合はNone。

        About:
            Receiverからraw frameを取得し、application responseへ復号する。
        """
        try:
            raw = self._receiver.read()
        except Exception:  # noqa: BLE001
            self._logger.warning("[ROBOT] IM920の受信処理に失敗しました", exc_info=True)
            return None
        if not raw:
            return None
        self._logger.debug("[ROBOT] IM920からraw frameを受信しました: %r", raw)
        return decode_frame(raw)

    def close(self) -> None:
        """
        Args:
            なし。

        Returns:
            なし。

        About:
            IM920-HAT deviceが所有するhardware resourceを解放する。
        """
        self._device.close()


def create_im920_client(
    *,
    logger: logging.Logger | None = None,
) -> IM920Client:
    """
    Args:
        logger: Clientが通信ログに使用するlogger。

    Returns:
        実機IM920-HAT driverを使用するClient。

    About:
        hardware依存moduleを遅延importし、実機通信用Clientを生成する。
    """
    # Hardware依存moduleはfake Clientのテスト時にRPi.GPIOを要求しないよう遅延importする。
    from unagifestival.tools.ps_controller.im920.driver import (  # noqa: PLC0415
        IM920HatDriver,
    )

    return IM920Client(IM920HatDriver(SLAVE_ADDRESS), logger=logger)
