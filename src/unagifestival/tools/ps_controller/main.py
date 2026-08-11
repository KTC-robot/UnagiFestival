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
from unagifestival.tools.ps_controller.models import AxisValueMap, ControllerState


def setup_logger() -> logging.Logger:
    """ロガーを初期化する.

    実行日時をファイル名にしてログを保存する。
    """
    logger = logging.getLogger("unagi_log")
    logger.setLevel(logging.INFO)

    # 多重登録防止
    if logger.handlers:
        return logger

    timestamp = datetime.now(UTC).astimezone().strftime("%Y%m%d_%H%M%S")
    log_filename = f"unagi_log_{timestamp}.log"

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
    if dev is None:
        logger.error("コントローラーが見つからないため終了します。")
        return

    axis_info = get_absolute_axis_info(dev)

    logger.info("Controller: %s %s", dev.path, dev.name)

    axis_values: AxisValueMap = {}

    for code, info in axis_info.items():
        if info.value is not None:
            axis_values[code] = info.value
        else:
            axis_values[code] = (info.min + info.max) // 2

    state = ControllerState(axis_values=axis_values, axis_info=axis_info)

    handler = RobotHandler()
    handler.enter()

    last_send = 0.0

    with contextlib.suppress(OSError):
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
                        state.axis_values[ev.code] = ev.value
                        handler.handle_abs(ev.code, state)

                    elif ev.type == EventType.KEY:
                        handler.handle_key(ev.code, ev.value)

            last_send = handler.tick(now, state, last_send)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt")

    finally:
        handler.exit()

        with contextlib.suppress(OSError):
            dev.ungrab()

        logger.info("=== CONTROLLER END ===")


if __name__ == "__main__":
    main()
