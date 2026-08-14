from dataclasses import dataclass
from enum import IntEnum

from unagifestival.tools.ps_controller.enums import AxisCode, ButtonCode


@dataclass(frozen=True)
class AxisInfo:
    """コントローラー軸の現在値と範囲情報."""

    value: int | None
    minimum: int
    maximum: int


type AxisValueMap = dict[AxisCode, int]
type AxisInfoMap = dict[AxisCode, AxisInfo]


@dataclass
class ControllerState:
    """実行中に更新されるコントローラー軸状態."""

    axis_values: AxisValueMap
    axis_info: AxisInfoMap


@dataclass(frozen=True)
class DriveCommand:
    """正規化したロボット移動指令."""

    vx: int
    vy: int
    wz: int


@dataclass(frozen=True)
class ServoSetCommand:
    """サーボの出力先と角度を表す指令."""

    channel: int
    angle: int


class StepperCommand(IntEnum):
    """2台のステッピングモーターに対する動作指令."""

    STOP = 0
    UP = 1
    DOWN = 2


@dataclass(frozen=True)
class ServoChannelConfig:
    """1つのPCA9685チャネルに対するサーボ設定."""

    enabled: bool
    min_angle: int
    max_angle: int
    home_angle: int


@dataclass(frozen=True)
class ServoAction:
    """ボタン押下時に実行する固定角度のサーボ操作."""

    channel: int
    angle: int


@dataclass(frozen=True)
class ServoToggleAction:
    """ボタン押下ごとに2つの角度を切り替えるサーボ操作."""

    channel: int
    angle_a: int
    angle_b: int


@dataclass(frozen=True)
class ServoToggleKey:
    """ボタン内のtoggle actionを識別するキー."""

    button: ButtonCode
    action_index: int


type ServoActionMap = dict[ButtonCode, tuple[ServoAction, ...]]
type ServoToggleActionMap = dict[ButtonCode, tuple[ServoToggleAction, ...]]
