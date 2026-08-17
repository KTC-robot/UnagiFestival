from dataclasses import dataclass

from unagifestival.tools.ps_controller.enum import (
    AxisCode,
    ButtonCode,
    ButtonState,
)


@dataclass(frozen=True)
class AxisInfo:
    """
    Properties:
        value: Controller接続時の軸値。
        minimum: 軸が取り得る最小値。
        maximum: 軸が取り得る最大値。

    About:
        Controller軸の初期値と正規化に必要な入力範囲を保持する。
    """

    value: int | None
    minimum: int
    maximum: int


type AxisValueMap = dict[AxisCode, int]
type AxisInfoMap = dict[AxisCode, AxisInfo]


@dataclass
class ControllerState:
    """
    Properties:
        axis_values: 軸ごとの最新入力値。
        axis_info: 軸ごとの入力範囲情報。

    About:
        Handlerが走行Commandを生成するためのController状態を保持する。
    """

    axis_values: AxisValueMap
    axis_info: AxisInfoMap


@dataclass(frozen=True)
class AxisInputEvent:
    """
    Properties:
        code: 入力されたController軸。
        value: 軸のraw入力値。

    About:
        evdevの絶対軸入力をapplicationで扱うイベントとして保持する。
    """

    code: AxisCode
    value: int


@dataclass(frozen=True)
class ButtonEvent:
    """
    Properties:
        code: 入力されたControllerボタン。
        state: ボタンの押下状態。

    About:
        evdevのキー入力をapplicationで扱うイベントとして保持する。
    """

    code: ButtonCode
    state: ButtonState


type ControllerInputEvent = AxisInputEvent | ButtonEvent
