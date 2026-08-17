from dataclasses import dataclass

from unagifestival.tools.ps_controller.enum import ButtonCode


@dataclass(frozen=True)
class ServoSetCommand:
    """1つの論理サーボへ角度を設定する指令."""

    channel: int
    angle: int


@dataclass(frozen=True)
class ServoSetAllCommand:
    """全論理サーボへ同じ角度を設定する指令."""

    angle: int


@dataclass(frozen=True)
class ServoChannelConfig:
    """1つの論理サーボに対する角度設定."""

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
    """ボタン押下ごとに2つの角度を切り替える操作."""

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
type ServoCommand = ServoSetCommand | ServoSetAllCommand
