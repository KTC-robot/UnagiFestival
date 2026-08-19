from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class StopCommand:
    """
    Properties:
        なし。

    About:
        車体の通常停止を要求する意味Commandを表す。
    """


@dataclass(frozen=True)
class EmergencyStopCommand:
    """
    Properties:
        なし。

    About:
        車体の緊急停止を要求する意味Commandを表す。
    """


@dataclass(frozen=True)
class ChangePowerCommand:
    """
    Properties:
        delta: 走行出力率へ加える相対値。

    About:
        現在の走行出力率を相対変更する意味Commandを保持する。
    """

    delta: int


@dataclass(frozen=True)
class DriveCommand:
    """
    Properties:
        vx: 前後方向の移動指令値。
        vy: 左右方向の移動指令値。
        wz: 回転方向の移動指令値。

    About:
        Controller入力から生成された正規化済みの車体走行Commandを保持する。
    """

    vx: int
    vy: int
    wz: int


@dataclass(frozen=True)
class SetWheelGainCommand:
    """
    Properties:
        direction: 補正対象の走行方向番号。
        wheel: 補正対象の車輪番号。
        gain: 目標RPMへ掛ける補正係数。

    About:
        方向および車輪別のRPM補正gainを設定する意味Commandを保持する。
    """

    direction: int
    wheel: int
    gain: float


@dataclass(frozen=True)
class GainTuneStartCommand:
    """
    Properties:
        drive: 測定中に使用する走行Command。
        duration_ms: 測定時間をミリ秒で表した値。

    About:
        wheel gain測定走行の開始条件を保持する。
    """

    drive: DriveCommand
    duration_ms: int


@dataclass(frozen=True)
class GainTuneKeepaliveCommand:
    """
    Properties:
        なし。

    About:
        wheel gain測定中に通信継続を通知する意味Commandを表す。
    """


@dataclass(frozen=True)
class GainTuneResultAckCommand:
    """
    Properties:
        result_index: 受信を確認した結果frameの番号。

    About:
        wheel gain測定結果に対するapplication ACKを保持する。
    """

    result_index: int


@dataclass(frozen=True)
class StepAssistResetCommand:
    """
    Properties:
        なし。

    About:
        ESP32の段差制御状態をresetする意味Commandを表す。
    """


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
)


@dataclass(frozen=True)
class EncodedPacket:
    """
    Properties:
        payload: TXDAへ渡す16進数文字列。
        label: ログ表示でpacketを識別する名称。

    About:
        Encoderが生成しTransmitterへ渡す送信packetを保持する。
    """

    payload: str
    label: str


@dataclass(frozen=True)
class IM920Response:
    """
    Properties:
        raw: IM920-HATから取得したraw frame。
        text: applicationで利用する復号済み文字列。

    About:
        Decoderによる復号前後の受信データを対応付けて保持する。
    """

    raw: str
    text: str


class IM920Device(Protocol):
    """
    Properties:
        なし。

    About:
        TransmitterとReceiverが利用するIM920-HAT低レイヤinterfaceを定義する。
    """

    def write(self, command: str) -> None:
        """
        Args:
            command: IM920-HATへ書き込むcommand文字列。

        Returns:
            なし。

        About:
            低レイヤ通信を利用してIM920-HATへcommandを送る。
        """
        ...

    def read(self) -> str:
        """
        Args:
            なし。

        Returns:
            受信済みのraw frame。データがない場合は空文字列。

        About:
            IM920-HATから受信可能なframeを1件取得する。
        """
        ...

    def close(self) -> None:
        """
        Args:
            なし。

        Returns:
            なし。

        About:
            低レイヤ通信で所有するhardware resourceを解放する。
        """
        ...
