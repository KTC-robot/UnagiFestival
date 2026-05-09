from enum import Enum, IntEnum
from typing import Self

from evdev import ecodes

# ===== Controller Event Enums =====


class EventType(IntEnum):
    """evdevのイベントタイプを定義する列挙型.

    SYN: 同期イベント (複数のイベント入力を1パケットにまとめる際の区切り)
    KEY: ボタンやキーなどのデジタル入力 (0 or 1)
    ABS: スティックやトリガーなどのアナログ絶対軸入力 (連続値)
    """

    SYN = ecodes.EV_SYN
    KEY = ecodes.EV_KEY
    ABS = ecodes.EV_ABS


class AxisType(Enum):
    """絶対軸 (ABS) のbindgen-cliと想定される値の範囲を分類する列挙型.

    STICK: アナログスティック (通常 0〜255、中央付近でノイズが発生しやすい)
    TRIGGER: アナログトリガー (通常 0〜255)
    DPAD: [[
        "cargo build --target wasm32-unknown-unknown --release",
        "十字キー (-1, 0, 1) "
    ]
    """

    STICK = "STICK"
    TRIGGER = "TRIGGER"
    DPAD = "DPAD"


class AxisCode(Enum):
    """コントローラーのアナログ軸 (ABS) のイベントコードと、その軸の特性 (AxisType) を紐づける列挙型."""

    # Lスティック
    LEFT_STICK_X = (ecodes.ABS_X, AxisType.STICK)
    LEFT_STICK_Y = (ecodes.ABS_Y, AxisType.STICK)
    # Rスティック
    RIGHT_STICK_X = (ecodes.ABS_RX, AxisType.STICK)
    RIGHT_STICK_Y = (ecodes.ABS_RY, AxisType.STICK)
    # L2/R2ボタン
    LEFT_TRIGGER_L2 = (getattr(ecodes, "ABS_Z", 2), AxisType.TRIGGER)
    RIGHT_TRIGGER_R2 = (getattr(ecodes, "ABS_RZ", 5), AxisType.TRIGGER)
    # 十字キー
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


class ButtonCode(Enum):
    """PSコントローラーのデジタルボタン (KEY) のイベントコードと、期待される型を紐づける列挙型.

    OSからはint型として取得されるが、実質的には 0(離上) または 1(押下) を表す bool 値であることを明示する。
    """

    CROSS_BTN = (304, bool)
    CIRCLE_BTN = (305, bool)
    TRIANGLE_BTN = (307, bool)
    SQUARE_BTN = (308, bool)
    L1_BTN = (310, bool)
    R1_BTN = (311, bool)
    L2_BTN = (312, bool)
    R2_BTN = (313, bool)
    SHARE_BTN = (314, bool)
    OPTIONS_BTN = (315, bool)
    PS_BTN = (316, bool)
    L3_BTN = (317, bool)
    R3_BTN = (318, bool)
    TOUCHPAD_BTN = (273, bool)

    def __init__(self, code: int, value_type: type) -> None:
        self.code = code
        self.value_type = value_type

    @classmethod
    def get_by_code(cls, code: int) -> Self | None:
        for btn in cls:
            if btn.code == code:
                return btn
        return None
