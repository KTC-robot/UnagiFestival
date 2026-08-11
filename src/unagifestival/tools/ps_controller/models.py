from dataclasses import dataclass
from typing import Protocol

from unagifestival.tools.ps_controller.enums import ButtonCode


class AxisInfo(Protocol):
    """コントローラー軸の正規化に必要な範囲情報."""

    @property
    def value(self) -> int | None: ...

    @property
    def min(self) -> int: ...

    @property
    def max(self) -> int: ...


type EventCode = int
type AxisValueMap = dict[EventCode, int]
type AxisInfoMap = dict[EventCode, AxisInfo]


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
