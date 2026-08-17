from dataclasses import dataclass

from unagifestival.tools.ps_controller.enum import ButtonCode


@dataclass(frozen=True)
class ServoSetCommand:
    """
    Properties:
        channel: 操作対象の論理Servo channel。
        angle: 設定する角度。

    About:
        1つの論理Servoへ角度を設定するCommandを保持する。
    """

    channel: int
    angle: int


@dataclass(frozen=True)
class ServoSetAllCommand:
    """
    Properties:
        angle: 全論理Servoへ設定する角度。

    About:
        すべての論理Servoへ同じ角度を設定するCommandを保持する。
    """

    angle: int


@dataclass(frozen=True)
class ServoChannelConfig:
    """
    Properties:
        enabled: channelが操作対象かどうか。
        min_angle: 設定可能な最小角度。
        max_angle: 設定可能な最大角度。
        home_angle: 起動時に使用する基準角度。

    About:
        1つの論理Servo channelに対する有効状態と角度範囲を保持する。
    """

    enabled: bool
    min_angle: int
    max_angle: int
    home_angle: int


@dataclass(frozen=True)
class ServoAction:
    """
    Properties:
        channel: 操作対象の論理Servo channel。
        angle: ボタン押下時に設定する角度。

    About:
        Controllerボタンへ割り当てる固定角度のServo操作を保持する。
    """

    channel: int
    angle: int


@dataclass(frozen=True)
class ServoToggleAction:
    """
    Properties:
        channel: 操作対象の論理Servo channel。
        angle_a: 交互操作の1つ目の角度。
        angle_b: 交互操作の2つ目の角度。

    About:
        Controllerボタンを押すたびに切り替える2つのServo角度を保持する。
    """

    channel: int
    angle_a: int
    angle_b: int


@dataclass(frozen=True)
class ServoToggleKey:
    """
    Properties:
        button: toggle操作を割り当てたControllerボタン。
        action_index: 同一ボタン内の操作番号。

    About:
        ServoMapperがtoggle状態を操作単位で識別するためのキーを保持する。
    """

    button: ButtonCode
    action_index: int


type ServoActionMap = dict[ButtonCode, tuple[ServoAction, ...]]
type ServoToggleActionMap = dict[ButtonCode, tuple[ServoToggleAction, ...]]
type ServoCommand = ServoSetCommand | ServoSetAllCommand
