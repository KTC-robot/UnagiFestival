import logging

from typing import Protocol

from unagifestival.tools.ps_controller.im920.constants import (
    IM920_COMMAND_MAX_LENGTH,
    SLAVE_ADDRESS,
)
from unagifestival.tools.ps_controller.im920.decoder import decode_frame
from unagifestival.tools.ps_controller.im920.encoder import encode_command
from unagifestival.tools.ps_controller.im920.factory import CommandFactory
from unagifestival.tools.ps_controller.im920.model import (
    IM920Command,
    IM920Device,
    IM920Response,
)
from unagifestival.tools.ps_controller.im920.receiver import IM920Receiver
from unagifestival.tools.ps_controller.im920.transmitter import IM920Transmitter


class IM920ClientProtocol(Protocol):
    """Handlerが依存するIM920 Facade interface."""

    commands: CommandFactory

    def send(self, command: IM920Command) -> None: ...

    def poll(self) -> IM920Response | None: ...

    def close(self) -> None: ...


class IM920Client:
    """Command送信とapplication response受信を公開するFacade."""

    def __init__(
        self,
        device: IM920Device,
        *,
        command_max_length: int = IM920_COMMAND_MAX_LENGTH,
        logger: logging.Logger | None = None,
    ) -> None:
        self._device = device
        self._logger = logger or logging.getLogger("unagi_log")
        self._transmitter = IM920Transmitter(
            device,
            command_max_length,
            self._logger,
        )
        self._receiver = IM920Receiver(device)
        self.commands = CommandFactory()

    def send(self, command: IM920Command) -> None:
        """意味CommandをencodeしてIM920-HATへ送信する."""
        self._transmitter.send(encode_command(command))

    def poll(self) -> IM920Response | None:
        """受信frameを最大1件取得しapplication responseへ復号する."""
        try:
            raw = self._receiver.read()
        except Exception:  # noqa: BLE001
            self._logger.warning("[ROBOT] IM920 read failed", exc_info=True)
            return None
        if not raw:
            return None
        self._logger.debug("[ROBOT] IM920 <- %r", raw)
        return decode_frame(raw)

    def close(self) -> None:
        """IM920-HATのhardware resourceを解放する."""
        self._device.close()


def create_im920_client(
    *,
    logger: logging.Logger | None = None,
) -> IM920Client:
    """実機IM920-HAT driverを使用するClientを生成する."""
    # Hardware依存moduleはfake Clientのテスト時にRPi.GPIOを要求しないよう遅延importする。
    from unagifestival.tools.ps_controller.im920.driver import (  # noqa: PLC0415
        IM920HatDriver,
    )

    return IM920Client(IM920HatDriver(SLAVE_ADDRESS), logger=logger)
