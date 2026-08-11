import contextlib
import logging
import time

from datetime import UTC, datetime

from unagifestival.tools.ps_controller.device import (
    find_controller,
    get_absolute_axis_info,
    wait_for_input_ready,
)
from unagifestival.tools.ps_controller.enums import EventType
from unagifestival.tools.ps_controller.handler import RobotHandler


def setup_logger() -> logging.Logger:
    """
    ロガーの初期設定。
    実行日時をファイル名にしてログを保存する。
    """
    logger = logging.getLogger("teensy_log")
    logger.setLevel(logging.INFO)

    # 多重登録防止
    if logger.handlers:
        return logger

    log_filename = f"teensy_log_{datetime.now(UTC).astimezone().strftime('%Y%m%d_%H%M%S')}.log"

    file_handler = logging.FileHandler(log_filename)
    file_handler.setFormatter(logging.Formatter("%(asctime)s - %(message)s"))

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(logging.Formatter("%(asctime)s - %(message)s"))

    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    return logger


def main() -> None:
    logger = setup_logger()
    logger.info("=== CONTROLLER START ===")

    dev = find_controller()
    info = get_absolute_axis_info(dev)

    logger.info("Controller: %s %s", dev.path, dev.name)

    raw = {}

    for code, axis_info in info.items():
        if axis_info is not None and axis_info.value is not None:
            raw[code] = axis_info.value
        elif axis_info is not None:
            raw[code] = (axis_info.min + axis_info.max) // 2
        else:
            raw[code] = 0

    handler = RobotHandler()
    handler.enter()

    last_send = 0.0

    with contextlib.suppress(Exception):
        dev.grab()

    try:
        while True:
            now = time.time()

            readable, _, _ = wait_for_input_ready(
                [dev.fd],
                timeout_seconds=0.005,
            )

            if readable:
                for ev in dev.read():
                    if ev.type == EventType.ABS:
                        raw[ev.code] = ev.value
                        handler.handle_abs(ev.code, raw, info)

                    elif ev.type == EventType.KEY:
                        handler.handle_key(ev.code, ev.value)

            last_send = handler.tick(now, raw, info, last_send)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt")

    finally:
        handler.exit()

        with contextlib.suppress(Exception):
            dev.ungrab()

        logger.info("=== CONTROLLER END ===")


if __name__ == "__main__":
    main()