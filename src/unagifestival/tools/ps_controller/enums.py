from enum import Enum, IntEnum
from typing import Self

from evdev import ecodes


# ============================================================
# Controller event types
# ============================================================


class EventType(IntEnum):
    """evdevのイベントタイプ。"""

    SYN = ecodes.EV_SYN
    KEY = ecodes.EV_KEY
    ABS = ecodes.EV_ABS


class AxisType(Enum):
    """ABSイベントの軸種別。"""

    STICK = "STICK"
    TRIGGER = "TRIGGER"
    DPAD = "DPAD"


class AxisCode(Enum):
    """アナログ軸のevdevコードと軸種別。"""

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


# ============================================================
# PS5 button table
# ============================================================


class ButtonCode(Enum):
    """
    PS5ボタンの対応表。

    ここだけを編集すれば、
      ・Linux evdevコード
      ・IM920で送るbutton_id
      ・ログに表示するボタン名
    の対応を一括管理できる。

    tuple:
        (evdev_code, packet_id)

    例:
        SQUARE_BTN = (308, 3)
    """

    CROSS_BTN = (304, 0)
    CIRCLE_BTN = (305, 1)
    TRIANGLE_BTN = (307, 2)
    SQUARE_BTN = (308, 3)
    L1_BTN = (310, 4)
    R1_BTN = (311, 5)
    L2_BTN = (312, 6)
    R2_BTN = (313, 7)
    SHARE_BTN = (314, 8)
    OPTIONS_BTN = (315, 9)
    PS_BTN = (316, 10)
    L3_BTN = (317, 11)
    R3_BTN = (318, 12)
    TOUCHPAD_BTN = (273, 13)

    def __init__(self, code: int, packet_id: int) -> None:
        self.code = code
        self.packet_id = packet_id

        # 既存コードとの互換性のため残す。
        self.value_type = bool

    @property
    def display_name(self) -> str:
        """ログ表示用の名前。例: SQUARE_BTN -> SQUARE"""
        return self.name.removesuffix("_BTN")

    @classmethod
    def get_by_code(cls, code: int) -> Self | None:
        """Linuxのevdevコードからボタンを取得する。"""
        for button in cls:
            if button.code == code:
                return button

        return None

    @classmethod
    def get_by_packet_id(cls, packet_id: int) -> Self | None:
        """IM920で送るbutton_idからボタンを取得する。"""
        for button in cls:
            if button.packet_id == packet_id:
                return button

        return None

    @classmethod
    def valid_packet_ids(cls) -> set[int]:
        return {button.packet_id for button in cls}