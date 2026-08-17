import contextlib
import logging
import time

from datetime import UTC, datetime
from pathlib import Path

from unagifestival.tools.ps_controller.device import (
    find_controller,
)
from unagifestival.tools.ps_controller.handler import Handler
from unagifestival.tools.ps_controller.model import (
    AxisInputEvent,
    AxisValueMap,
    ButtonEvent,
    ControllerState,
)


def setup_logger() -> logging.Logger:
    """
    Args:
        なし。

    Returns:
        Controller処理で共有するlogger。

    About:
        console出力とlogs配下の実行別ファイル出力を設定する。
        既にhandlerが登録済みの場合は重複登録しない。
    """
    logger = logging.getLogger("unagi_log")
    logger.setLevel(logging.INFO)

    # 多重登録防止
    if logger.handlers:
        return logger

    timestamp = datetime.now(UTC).astimezone().strftime("%Y%m%d_%H%M%S")
    logs_dir = Path("logs")
    logs_dir.mkdir(parents=True, exist_ok=True)
    log_filename = logs_dir / f"ps_controller_{timestamp}.log"

    file_handler = logging.FileHandler(log_filename)
    formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    file_handler.setFormatter(formatter)

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    return logger


def main() -> None:
    """
    Args:
        なし。

    Returns:
        なし。

    About:
        ControllerとHandlerを初期化し、入力処理と周期処理を実行する。
        終了時には通信resourceと入力デバイスの排他状態を解放する。
    """
    logger = setup_logger()
    logger.info("=== Controller処理を開始します ===")

    dev = find_controller()
    if dev is None:
        logger.error("コントローラーが見つからないため終了します。")
        return

    axis_info = dev.axis_info

    logger.info("Controllerを使用します: path=%s name=%s", dev.path, dev.name)

    axis_values: AxisValueMap = {}

    for code, info in axis_info.items():
        if info.value is not None:
            axis_values[code] = info.value
        else:
            axis_values[code] = (info.minimum + info.maximum) // 2

    state = ControllerState(axis_values=axis_values, axis_info=axis_info)

    handler = Handler()
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
        logger.info("キーボード割り込みを受け付けました")

    finally:
        handler.exit()

        with contextlib.suppress(OSError):
            dev.ungrab()

        logger.info("=== Controller処理を終了します ===")


if __name__ == "__main__":
    main()
