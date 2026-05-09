import contextlib
import logging
import time

from datetime import UTC, datetime

from unagifestival.tools.ps_controller.config import PORT
from unagifestival.tools.ps_controller.device import (
    find_controller,
    get_absolute_axis_info,
    wait_for_input_ready,
)
from unagifestival.tools.ps_controller.enums import EventType

# ★ 各種モードの代わりに、入力を一手に引き受ける統合ハンドラをインポート
from unagifestival.tools.ps_controller.handler import RobotHandler


def setup_logger() -> logging.Logger:
    """ロガーの初期設、実行日時をファイル名にしてログを保存する."""
    logger = logging.getLogger("teensy_log")
    logger.setLevel(logging.INFO)
    log_filename = f"teensy_log_{datetime.now(UTC).astimezone().strftime('%Y%m%d_%H%M%S')}.log"
    fh = logging.FileHandler(log_filename)
    fh.setFormatter(logging.Formatter("%(asctime)s - %(message)s"))
    logger.addHandler(fh)
    return logger


def main() -> None:
    logger = setup_logger()
    logger.info("=== CONTROLLER START ===")

    dev = find_controller()
    info = get_absolute_axis_info(dev)
    logger.info("Controller: %s %s", dev.path, dev.name)

    raw = {}
    for code, ai in info.items():
        raw[code] = ai.value if ai and ai.value is not None else (ai.min + ai.max) // 2

    logger.info("Open UART: %d", PORT)

    handler = RobotHandler()
    handler.enter()
    last_send = 0.0

    with contextlib.suppress(Exception):
        dev.grab()

    try:
        while True:
            now = time.time()
            r, _, _ = wait_for_input_ready([dev.fd], timeout_seconds=0.005)

            if r:
                for ev in dev.read():
                    # イベント種別に応じてハンドラへ丸投げ
                    if ev.type == EventType.ABS:
                        raw[ev.code] = ev.value
                        handler.handle_abs(ev.code, ev.value)

                    elif ev.type == EventType.KEY:
                        handler.handle_key(ev.code, ev.value)

            # 周期処理もハンドラへ委譲
            last_send = handler.tick(now, raw, info, last_send)

    except KeyboardInterrupt:
        pass
    finally:
        handler.exit()
        with contextlib.suppress(Exception):
            dev.ungrab()


if __name__ == "__main__":
    main()
