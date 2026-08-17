import logging

from unagifestival.tools.ps_controller.enum import ButtonCode
from unagifestival.tools.ps_controller.servo.constants import (
    SERVO_ANGLE_MAX,
    SERVO_BUTTON_ACTIONS,
    SERVO_CHANNEL_COUNT,
    SERVO_CHANNELS,
    SERVO_TOGGLE_ACTIONS,
)
from unagifestival.tools.ps_controller.servo.model import ServoSetCommand

logger = logging.getLogger("unagi_log")

DRIVE_RESERVED_BUTTONS = {
    ButtonCode.CROSS_BTN,
    ButtonCode.CIRCLE_BTN,
    ButtonCode.L1_BTN,
    ButtonCode.R1_BTN,
    ButtonCode.PS_BTN,
}


def validate_servo_target(button: ButtonCode, command: ServoSetCommand) -> None:
    """ボタン割り当ての論理チャネルと角度を検証する."""
    if not 0 <= command.channel < SERVO_CHANNEL_COUNT:
        message = (
            f"Invalid servo channel: button={button.display_name} "
            f"channel={command.channel}"
        )
        raise ValueError(message)

    config = SERVO_CHANNELS[command.channel]
    if not config.enabled:
        message = (
            f"Servo CH{command.channel} is assigned to button "
            f"{button.display_name}, but the channel is disabled."
        )
        raise ValueError(message)
    if not config.min_angle <= command.angle <= config.max_angle:
        message = (
            f"Servo angle is outside the configured range: "
            f"button={button.display_name} CH{command.channel} "
            f"angle={command.angle} range={config.min_angle}..{config.max_angle}"
        )
        raise ValueError(message)


def _validate_actions() -> None:
    for button, actions in SERVO_BUTTON_ACTIONS.items():
        if button in DRIVE_RESERVED_BUTTONS:
            logger.warning(
                "[SERVO] Button %s is also used by chassis control.",
                button.display_name,
            )
        for action in actions:
            validate_servo_target(
                button,
                ServoSetCommand(action.channel, action.angle),
            )

    for button, actions in SERVO_TOGGLE_ACTIONS.items():
        if button in DRIVE_RESERVED_BUTTONS:
            logger.warning(
                "[SERVO] Button %s is also used by chassis control.",
                button.display_name,
            )
        for action in actions:
            validate_servo_target(
                button,
                ServoSetCommand(action.channel, action.angle_a),
            )
            validate_servo_target(
                button,
                ServoSetCommand(action.channel, action.angle_b),
            )

    duplicate_buttons = set(SERVO_BUTTON_ACTIONS) & set(SERVO_TOGGLE_ACTIONS)
    for button in sorted(duplicate_buttons, key=lambda item: item.code):
        logger.warning(
            "[SERVO] Button %s has both direct and toggle actions.",
            button.display_name,
        )


def validate_servo_config() -> None:
    """サーボ設定と全ボタンmappingを検証する."""
    if len(SERVO_CHANNELS) != SERVO_CHANNEL_COUNT:
        message = (
            f"SERVO_CHANNELS must contain {SERVO_CHANNEL_COUNT} values, "
            f"but contains {len(SERVO_CHANNELS)}"
        )
        raise ValueError(message)

    for channel, config in enumerate(SERVO_CHANNELS):
        if not 0 <= config.min_angle <= config.max_angle <= SERVO_ANGLE_MAX:
            message = (
                f"Invalid servo angle range: CH{channel} "
                f"min={config.min_angle} max={config.max_angle}"
            )
            raise ValueError(message)
        if not config.min_angle <= config.home_angle <= config.max_angle:
            message = (
                f"Servo home angle is outside the range: CH{channel} "
                f"home={config.home_angle} "
                f"range={config.min_angle}..{config.max_angle}"
            )
            raise ValueError(message)

    _validate_actions()
