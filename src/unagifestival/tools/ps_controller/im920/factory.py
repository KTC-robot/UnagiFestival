from unagifestival.tools.ps_controller.im920.model import (
    ChangePowerCommand,
    DriveCommand,
    EmergencyStopCommand,
    GainTuneKeepaliveCommand,
    GainTuneResultAckCommand,
    GainTuneStartCommand,
    SetWheelGainCommand,
    StepAssistResetCommand,
    StopCommand,
)


class CommandFactory:
    """
    Properties:
        なし。
    About:
        HandlerやCLIの意味操作からwire形式に依存しないCommandを生成する。
    """

    @staticmethod
    def drive(vx: int, vy: int, wz: int) -> DriveCommand:
        """
        Args:
            vx: 前後方向の指令値。
            vy: 左右方向の指令値。
            wz: 回転方向の指令値。
        Returns:
            走行Command。
        About:
            3軸の車体走行指令を生成する。
        """
        return DriveCommand(vx, vy, wz)

    @staticmethod
    def stop() -> StopCommand:
        """
        Args:
            なし。
        Returns:
            通常停止Command。
        About:
            車体の通常停止指令を生成する。
        """
        return StopCommand()

    @staticmethod
    def emergency_stop() -> EmergencyStopCommand:
        """
        Args:
            なし。
        Returns:
            緊急停止Command。
        About:
            車体の緊急停止指令を生成する。
        """
        return EmergencyStopCommand()

    @staticmethod
    def change_power(delta: int) -> ChangePowerCommand:
        """
        Args:
            delta: 走行出力率へ加える相対値。
        Returns:
            走行出力率変更Command。
        About:
            現在値に対する走行出力率の変更指令を生成する。
        """
        return ChangePowerCommand(delta)

    @staticmethod
    def reset_step_assist() -> StepAssistResetCommand:
        """
        Args:
            なし。
        Returns:
            段差制御reset Command。
        About:
            段差制御状態の初期化指令を生成する。
        """
        return StepAssistResetCommand()

    @staticmethod
    def set_wheel_gain(
        direction: int,
        wheel: int,
        gain: float,
    ) -> SetWheelGainCommand:
        """
        Args:
            direction: 補正対象の走行方向番号。
            wheel: 補正対象の車輪番号。
            gain: 設定するRPM補正係数。
        Returns:
            車輪補正gain設定Command。
        About:
            方向および車輪別の補正gain指令を生成する。
        """
        return SetWheelGainCommand(direction, wheel, gain)

    @staticmethod
    def start_gain_tuning(
        drive: DriveCommand,
        duration_ms: int,
    ) -> GainTuneStartCommand:
        """
        Args:
            drive: 測定中に使用する走行Command。
            duration_ms: 測定時間をミリ秒で表した値。
        Returns:
            gain測定開始Command。
        About:
            走行条件を含むgain測定開始指令を生成する。
        """
        return GainTuneStartCommand(drive, duration_ms)

    @staticmethod
    def gain_tuning_keepalive() -> GainTuneKeepaliveCommand:
        """
        Args:
            なし。
        Returns:
            gain測定keepalive Command。
        About:
            gain測定中の通信継続指令を生成する。
        """
        return GainTuneKeepaliveCommand()

    @staticmethod
    def ack_gain_tuning_result(result_index: int) -> GainTuneResultAckCommand:
        """
        Args:
            result_index: 受信を確認した結果frame番号。
        Returns:
            gain測定結果ACK Command。
        About:
            指定結果に対するapplication ACKを生成する。
        """
        return GainTuneResultAckCommand(result_index)
