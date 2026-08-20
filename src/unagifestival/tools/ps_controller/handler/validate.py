from unagifestival.tools.ps_controller.handler.constants import (
    AIR_TRIGGER_ACTIVE_RATIO,
    DRIVE_HZ,
    STATE_COMMAND_RETRY_COUNT,
    STATE_COMMAND_RETRY_INTERVAL_SECONDS,
    STICK_DEADZONE,
    STICK_SEND_MAX,
)

INVALID_HANDLER_CONFIG_MESSAGE = "invalid handler configuration"


def validate_handler_config() -> None:
    """
    Args:
        なし。
    Returns:
        なし。
    About:
        Handler固有の周期、入力範囲、状態Command再送設定が有効か検証する。
    """
    if (
        DRIVE_HZ <= 0
        or STICK_SEND_MAX <= 0
        or not 0.0 <= STICK_DEADZONE < 1.0
        or not 0.0 <= AIR_TRIGGER_ACTIVE_RATIO <= 1.0
        or STATE_COMMAND_RETRY_COUNT <= 0
        or STATE_COMMAND_RETRY_INTERVAL_SECONDS < 0.0
    ):
        raise ValueError(INVALID_HANDLER_CONFIG_MESSAGE)
