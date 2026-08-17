from unagifestival.tools.ps_controller.handler.constants import (
    DRIVE_HZ,
    SLOW_MODE_MULTIPLIER,
    STICK_DEADZONE,
    STICK_SEND_MAX,
    TRIGGER_ACTIVE_RATIO,
)

INVALID_HANDLER_CONFIG_MESSAGE = "invalid handler configuration"


def validate_handler_config() -> None:
    """Handler固有の周期・入力係数が安全な範囲か検証する."""
    if (
        DRIVE_HZ <= 0
        or STICK_SEND_MAX <= 0
        or not 0.0 <= STICK_DEADZONE < 1.0
        or not 0.0 < SLOW_MODE_MULTIPLIER <= 1.0
        or not 0.0 <= TRIGGER_ACTIVE_RATIO <= 1.0
    ):
        raise ValueError(INVALID_HANDLER_CONFIG_MESSAGE)
