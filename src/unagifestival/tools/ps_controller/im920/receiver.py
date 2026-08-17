from collections import deque

from unagifestival.tools.ps_controller.im920.model import IM920Device


class IM920Receiver:
    """IM920-HATからraw frameを欠落させず1件ずつ取り出す."""

    def __init__(self, device: IM920Device) -> None:
        self._device = device
        self._pending: deque[str] = deque()

    def read(self) -> str:
        """受信済みraw frameを最大1件返す."""
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
