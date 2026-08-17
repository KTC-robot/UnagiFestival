from dataclasses import dataclass

from unagifestival.tools.ps_controller.enum import (
    AxisCode,
    ButtonCode,
    ButtonState,
)


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
class AxisInputEvent:
    """軸種別と現在値を組み合わせた入力イベント."""

    code: AxisCode
    value: int


@dataclass(frozen=True)
class ButtonEvent:
    """ボタン種別と押下状態を組み合わせた入力イベント."""

    code: ButtonCode
    state: ButtonState


type ControllerInputEvent = AxisInputEvent | ButtonEvent
