"""IM920sL 920MHz無線シリアルモジュール 通信ラッパー."""

from __future__ import annotations

import logging
import threading
import warnings

from typing import Self

import serial

logger = logging.getLogger(__name__)

_MAX_PAYLOAD = 32  # IM920sL の最大ペイロード (ASCII モード)
_OK = b"OK"
_NG = b"NG"


class IM920sL:
    r"""IM920sL との UART 通信を抽象化するクラス.

    送信: TXDA コマンド (ASCII モード)
    受信フォーマット例: "0001,+65,Hello\r\n"
    """

    def __init__(self, port: str, baudrate: int = 19200, timeout: float = 0.1) -> None:
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self._lock = threading.Lock()
        logger.info("IM920sL opened: port=%s baudrate=%d", port, baudrate)

    # --- コンテキストマネージャ ---

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()
            logger.info("IM920sL closed")

    # --- 送信 ---

    def send(self, text: str) -> None:
        """ASCII テキストを IM920sL で無線送信する (32バイト超は切り捨て)."""
        encoded = text.encode("ascii", errors="replace")
        if len(encoded) > _MAX_PAYLOAD:
            """
            warnings.warn(
                f"IM920sL: payload {len(encoded)} bytes exceeds {_MAX_PAYLOAD} bytes, truncating",
                stacklevel=2,
            )
            """

            encoded = encoded[:_MAX_PAYLOAD]
            print("3")
        text_ascii = encoded.decode("ascii", errors="replace")
        print("2")
        cmd = b"TXDA " + encoded + b"\r\n"
        print("1")
        with self._lock:
            self._ser.write(cmd)
            resp = self._ser.readline().strip()
        if resp == _NG:
            logger.warning("TX NG: %r", text_ascii)
        elif resp != _OK:
            logger.debug("TX unexpected response: %r", resp)
        logger.debug("TX >> %s", text_ascii)

    # --- 受信 ---

    def recv(self) -> tuple[str, str, str] | None:
        r"""受信データを 1 行読み取り (node_id, rssi, data) で返す.

        受信データなし or タイムアウト時は None を返す。

        受信フォーマット例: "0001,+65,Hello\r\n"
        """
        with self._lock:
            line = self._ser.readline()
        if not line:
            return None
        decoded = line.decode("ascii", errors="replace").strip()
        parts = decoded.split(",", 2)
        if len(parts) != 3:  # noqa: PLR2004
            logger.debug("RX (unexpected format): %r", decoded)
            return None
        node_id, rssi, data = parts
        logger.debug("RX << node=%s rssi=%s data=%r", node_id, rssi, data)
        return node_id, rssi, data
