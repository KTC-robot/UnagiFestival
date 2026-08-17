from collections import deque

from unagifestival.tools.ps_controller.im920.model import IM920Device


class IM920Receiver:
    """
    Properties:
        なし。

    About:
        IM920-HATから取得したraw dataをframe単位で保持し、1件ずつ返す。
    """

    def __init__(self, device: IM920Device) -> None:
        """
        Args:
            device: raw dataを読み取るIM920 device。

        Returns:
            なし。

        About:
            読み取り元deviceと未返却frame用queueを初期化する。
        """
        self._device = device
        self._pending: deque[str] = deque()

    def read(self) -> str:
        """
        Args:
            なし。

        Returns:
            受信済みraw frame。データがない場合は空文字列。

        About:
            複数frameを分割して保持し、呼び出しごとに最大1件返す。
        """
        if self._pending:
            return self._pending.popleft()
        raw = self._device.read()
        if not raw:
            return ""
        frames = [frame for frame in raw.splitlines() if frame.strip()]
        if not frames:
            return ""
        self._pending.extend(frames[1:])
        return frames[0]
