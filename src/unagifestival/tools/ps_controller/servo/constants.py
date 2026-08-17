from typing import Final

from unagifestival.tools.ps_controller.enum import ButtonCode
from unagifestival.tools.ps_controller.servo.model import (
    ServoAction,
    ServoActionMap,
    ServoChannelConfig,
    ServoToggleAction,
    ServoToggleActionMap,
)

SERVO_CHANNEL_COUNT: Final[int] = 7
SERVO_ALL_OPEN_ANGLE: Final[int] = 180
SERVO_ALL_CLOSE_ANGLE: Final[int] = 0
SERVO_ANGLE_MAX: Final[int] = 180
SERVO_STARTUP_INTERVAL_SECONDS: Final[float] = 0.05
SERVO_SEND_HOME_ON_START: Final[bool] = False

SERVO_CHANNELS: Final[tuple[ServoChannelConfig, ...]] = tuple(
    ServoChannelConfig(enabled=True, min_angle=0, max_angle=180, home_angle=90)
    for _ in range(SERVO_CHANNEL_COUNT)
)

SERVO_BUTTON_ACTIONS: Final[ServoActionMap] = {
    ButtonCode.SQUARE_BTN: (ServoAction(channel=2, angle=60),),
    ButtonCode.TRIANGLE_BTN: (ServoAction(channel=2, angle=120),),
    ButtonCode.OPTIONS_BTN: (ServoAction(channel=1, angle=0),),
}

SERVO_TOGGLE_ACTIONS: Final[ServoToggleActionMap] = {
    ButtonCode.L2_BTN: (ServoToggleAction(channel=0, angle_a=0, angle_b=180),),
}
