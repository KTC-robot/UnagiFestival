from enum import Enum, IntEnum
from typing import Self

from evdev import ecodes


class EventType(IntEnum):
    """
    Properties:
        SYN: 同期イベントを表す。
        KEY: ボタンイベントを表す。
        ABS: 絶対軸イベントを表す。

    About:
        evdevから受信する入力イベントの種別を定義する。
    """

    SYN = ecodes.EV_SYN
    KEY = ecodes.EV_KEY
    ABS = ecodes.EV_ABS


class AxisType(Enum):
    """
    Properties:
        STICK: アナログスティック軸を表す。
        TRIGGER: アナログトリガー軸を表す。
        DPAD: 十字キー軸を表す。

    About:
        Controllerの絶対軸を操作方法ごとに分類する。
    """

    STICK = "STICK"
    TRIGGER = "TRIGGER"
    DPAD = "DPAD"


class AxisCode(Enum):
    """
    Properties:
        LEFT_STICK_X: 左スティックの左右軸。
        LEFT_STICK_Y: 左スティックの前後軸。
        RIGHT_STICK_X: 右スティックの左右軸。
        RIGHT_STICK_Y: 右スティックの前後軸。
        LEFT_TRIGGER_L2: 左トリガー軸。
        RIGHT_TRIGGER_R2: 右トリガー軸。
        DPAD_X: 十字キーの左右軸。
        DPAD_Y: 十字キーの上下軸。

    About:
        Controller軸のevdevコードと入力分類を対応付ける。
    """

    LEFT_STICK_X = (ecodes.ABS_X, AxisType.STICK)
    LEFT_STICK_Y = (ecodes.ABS_Y, AxisType.STICK)
    RIGHT_STICK_X = (ecodes.ABS_RX, AxisType.STICK)
    RIGHT_STICK_Y = (ecodes.ABS_RY, AxisType.STICK)
    LEFT_TRIGGER_L2 = (getattr(ecodes, "ABS_Z", 2), AxisType.TRIGGER)
    RIGHT_TRIGGER_R2 = (getattr(ecodes, "ABS_RZ", 5), AxisType.TRIGGER)
    DPAD_X = (ecodes.ABS_HAT0X, AxisType.DPAD)
    DPAD_Y = (ecodes.ABS_HAT0Y, AxisType.DPAD)

    def __init__(self, code: int, axis_type: AxisType) -> None:
        """
        Args:
            code: Linuxのevdev軸コード。
            axis_type: 軸の入力分類。

        Returns:
            なし。

        About:
            enum memberへevdevコードと軸分類を保持する。
        """
        self.code = code
        self.axis_type = axis_type

    @classmethod
    def get_by_code(cls, code: int) -> Self | None:
        """
        Args:
            code: 検索するLinuxのevdev軸コード。

        Returns:
            対応する軸が存在する場合はAxisCode、存在しない場合はNone。

        About:
            raw eventの軸コードをapplication側の軸定義へ変換する。
        """
        return next((axis for axis in cls if axis.code == code), None)


class ButtonState(IntEnum):
    """
    Properties:
        RELEASED: ボタンが離された状態。
        PRESSED: ボタンが押された状態。

    About:
        evdevが通知するボタンの押下状態を定義する。
    """

    RELEASED = 0
    PRESSED = 1


class ButtonCode(Enum):
    """
    Properties:
        CROSS_BTN: CROSSボタン。
        CIRCLE_BTN: CIRCLEボタン。
        TRIANGLE_BTN: TRIANGLEボタン。
        SQUARE_BTN: SQUAREボタン。
        L1_BTN: L1ボタン。
        R1_BTN: R1ボタン。
        L2_BTN: L2ボタン。
        R2_BTN: R2ボタン。
        SHARE_BTN: SHAREボタン。
        OPTIONS_BTN: OPTIONSボタン。
        PS_BTN: PSボタン。
        L3_BTN: L3ボタン。
        R3_BTN: R3ボタン。
        TOUCHPAD_BTN: タッチパッドボタン。

    About:
        PS Controllerの各ボタンをLinuxのevdevコードへ対応付ける。
    """

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
        """
        Args:
            なし。

        Returns:
            Linuxのevdevボタンコード。

        About:
            enum memberが保持する数値コードを整数として取得する。
        """
        return int(self.value)

    @property
    def display_name(self) -> str:
        """
        Args:
            なし。

        Returns:
            ログ表示用に接尾辞を除いたボタン名。

        About:
            Controller操作を読みやすい名称でログへ表示するために利用する。
        """
        return self.name.removesuffix("_BTN")

    @classmethod
    def get_by_code(cls, code: int) -> Self | None:
        """
        Args:
            code: 検索するLinuxのevdevボタンコード。

        Returns:
            対応するボタンが存在する場合はButtonCode、存在しない場合はNone。

        About:
            raw eventのボタンコードをapplication側のボタン定義へ変換する。
        """
        return next((button for button in cls if button.code == code), None)
