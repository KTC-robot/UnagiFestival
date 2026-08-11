from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Self

from evdev import ecodes

# ============================================================
# Controller event types
# ============================================================


class EventType(IntEnum):
    """evdevのイベントタイプ."""

    SYN = ecodes.EV_SYN
    KEY = ecodes.EV_KEY
    ABS = ecodes.EV_ABS


class AxisType(Enum):
    """ABSイベントの軸種別."""

    STICK = "STICK"
    TRIGGER = "TRIGGER"
    DPAD = "DPAD"


class AxisCode(Enum):
    """アナログ軸のevdevコードと軸種別."""

    LEFT_STICK_X = (ecodes.ABS_X, AxisType.STICK)
    LEFT_STICK_Y = (ecodes.ABS_Y, AxisType.STICK)

    RIGHT_STICK_X = (ecodes.ABS_RX, AxisType.STICK)
    RIGHT_STICK_Y = (ecodes.ABS_RY, AxisType.STICK)

    LEFT_TRIGGER_L2 = (
        getattr(ecodes, "ABS_Z", 2),
        AxisType.TRIGGER,
    )
    RIGHT_TRIGGER_R2 = (
        getattr(ecodes, "ABS_RZ", 5),
        AxisType.TRIGGER,
    )

    DPAD_X = (ecodes.ABS_HAT0X, AxisType.DPAD)
    DPAD_Y = (ecodes.ABS_HAT0Y, AxisType.DPAD)

    def __init__(self, code: int, axis_type: AxisType) -> None:
        self.code = code
        self.axis_type = axis_type

    @classmethod
    def get_by_code(cls, code: int) -> Self | None:
        for axis in cls:
            if axis.code == code:
                return axis

        return None


@dataclass(frozen=True)
class AxisInputEvent:
    """軸種別と現在値を組み合わせた入力イベント."""

    code: AxisCode
    value: int


# ============================================================
# PS5 button table
# ============================================================


class ButtonState(IntEnum):
    """evdevが通知するボタン状態."""

    RELEASED = 0
    PRESSED = 1


class ButtonCode(Enum):
    """PS5ボタンに対応するLinux evdevコード."""

    CROSS_BTN = ecodes.BTN_SOUTH
    CIRCLE_BTN = ecodes.BTN_EAST
    TRIANGLE_BTN = ecodes.BTN_NORTH
    SQUARE_BTN = ecodes.BTN_WEST
    L1_BTN = ecodes.BTN_TL
    R1_BTN = ecodes.BTN_TR
    L2_BTN = ecodes.BTN_TL2
    R2_BTN = ecodes.BTN_TR2
    SHARE_BTN = ecodes.BTN_SELECT
    OPTIONS_BTN = ecodes.BTN_START
    PS_BTN = ecodes.BTN_MODE
    L3_BTN = ecodes.BTN_THUMBL
    R3_BTN = ecodes.BTN_THUMBR
    TOUCHPAD_BTN = ecodes.BTN_RIGHT

    @property
    def code(self) -> int:
        """Linux evdevコードを返す."""
        return int(self.value)

    @property
    def display_name(self) -> str:
        """ログ表示用の名前。例: SQUARE_BTN -> SQUARE."""
        return self.name.removesuffix("_BTN")

    @classmethod
    def get_by_code(cls, code: int) -> Self | None:
        """Linuxのevdevコードからボタンを取得する."""
        for button in cls:
            if button.code == code:
                return button

        return None

@dataclass(frozen=True)
class ButtonEvent:
    """ボタン種別と押下状態を組み合わせた入力イベント."""

    code: ButtonCode
    state: ButtonState


type ControllerInputEvent = AxisInputEvent | ButtonEvent
