from dataclasses import dataclass
from typing import Protocol

from unagifestival.tools.ps_controller.servo.model import ServoCommand


@dataclass(frozen=True)
class StopCommand:
    """通常停止指令."""


@dataclass(frozen=True)
class EmergencyStopCommand:
    """緊急停止指令."""


@dataclass(frozen=True)
class ChangePowerCommand:
    """走行出力率の相対変更指令."""

    delta: int


@dataclass(frozen=True)
class DriveCommand:
    """正規化済みの車体走行指令."""

    vx: int
    vy: int
    wz: int


@dataclass(frozen=True)
class SetWheelGainCommand:
    """方向・車輪別RPM補正gain設定指令."""

    direction: int
    wheel: int
    gain: float


@dataclass(frozen=True)
class GainTuneStartCommand:
    """wheel gain測定走行の開始指令."""

    drive: DriveCommand
    duration_ms: int


@dataclass(frozen=True)
class GainTuneKeepaliveCommand:
    """wheel gain測定中のkeepalive指令."""


@dataclass(frozen=True)
class GainTuneResultAckCommand:
    """wheel gain測定結果のapplication ACK指令."""

    result_index: int


@dataclass(frozen=True)
class StepAssistResetCommand:
    """段差制御状態のreset指令."""


type IM920Command = (
    StopCommand
    | EmergencyStopCommand
    | ChangePowerCommand
    | DriveCommand
    | SetWheelGainCommand
    | GainTuneStartCommand
    | GainTuneKeepaliveCommand
    | GainTuneResultAckCommand
    | StepAssistResetCommand
    | ServoCommand
)


@dataclass(frozen=True)
class EncodedPacket:
    """TXDAへ渡すhex payloadとログ用label."""

    payload: str
    label: str


@dataclass(frozen=True)
class IM920Response:
    """IM920 raw frameと復号済みapplication text."""

    raw: str
    text: str


class IM920Device(Protocol):
    """transmitter/receiverが利用するIM920-HAT低レイヤinterface."""

    def write(self, command: str) -> None: ...

    def read(self) -> str: ...

    def close(self) -> None: ...
