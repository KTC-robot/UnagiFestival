import contextlib
import logging
import time

from datetime import UTC, datetime

from unagifestival.tools.ps_controller.device import (
    find_controller,
)
from unagifestival.tools.ps_controller.enums import AxisInputEvent, ButtonEvent
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

    axis_info = dev.axis_info

    logger.info("Controller: %s %s", dev.path, dev.name)

    axis_values: AxisValueMap = {}

    for code, info in axis_info.items():
        if info.value is not None:
            axis_values[code] = info.value
        else:
            axis_values[code] = (info.minimum + info.maximum) // 2

    state = ControllerState(axis_values=axis_values, axis_info=axis_info)

    handler = RobotHandler()
    handler.enter()

    last_send = 0.0

    with contextlib.suppress(OSError):
        dev.grab()

    try:
        while True:
            now = time.time()

            for event in dev.read_events(timeout_seconds=0.005):
                if isinstance(event, AxisInputEvent):
                    handler.handle_axis(event, state)
                elif isinstance(event, ButtonEvent):
                    handler.handle_button(event)

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
