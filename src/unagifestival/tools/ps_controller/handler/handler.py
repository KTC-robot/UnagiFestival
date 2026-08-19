import logging

from unagifestival.tools.ps_controller.enum import (
    AxisCode,
    ButtonCode,
    ButtonState,
)
from unagifestival.tools.ps_controller.handler.constants import (
    DRIVE_HZ,
    DRIVE_POWER_STEP,
    SLOW_MODE_MULTIPLIER,
    STICK_DEADZONE,
    STICK_SEND_MAX,
    TRIGGER_ACTIVE_RATIO,
)
from unagifestival.tools.ps_controller.handler.validate import (
    validate_handler_config,
)
from unagifestival.tools.ps_controller.im920.client import (
    IM920ClientProtocol,
    create_im920_client,
)
from unagifestival.tools.ps_controller.im920.factory import CommandFactory
from unagifestival.tools.ps_controller.im920.model import DriveCommand
from unagifestival.tools.ps_controller.model import (
    AxisInfo,
    AxisInputEvent,
    ButtonEvent,
    ControllerState,
)

logger = logging.getLogger("unagi_log")


class Handler:
    """
    Properties:
        im920: IM920との送受信を行うClient。
        commands: ロボット制御Commandを生成するFactory。

    About:
        Controller入力から走行、停止、StepAssist操作のCommandを生成して送信する。
        周期処理とIM920 responseの受信も管理する。
    """

    def __init__(
        self,
        im920: IM920ClientProtocol | None = None,
    ) -> None:
        """
        Args:
            im920: 注入するIM920 Client。未指定時はenterで実機Clientを生成する。

        Returns:
            なし。

        About:
            Handlerの依存関係とCommand生成器を保持し、設定を検証する。
        """
        validate_handler_config()
        self.im920 = im920
        self.commands = CommandFactory()

    def enter(self) -> None:
        """
        Args:
            なし。

        Returns:
            なし。

        About:
            必要に応じて実機Clientを生成し、Controller通信を開始する。
        """
        if self.im920 is None:
            self.im920 = create_im920_client(logger=logger)

        logger.info("[ROBOT] PS ControllerからIM920-HATへの送信処理を開始します")

    def exit(self) -> None:
        """
        Args:
            なし。

        Returns:
            なし。

        About:
            初期化済みの場合にIM920-HATのhardware resourceを解放する。
        """
        logger.info("[ROBOT] 制御処理を終了します")
        if self.im920 is None:
            return
        try:
            self.im920.close()
        except Exception:  # noqa: BLE001
            logger.warning("[ROBOT] IM920の終了処理に失敗しました", exc_info=True)

    def _get_im920(self) -> IM920ClientProtocol:
        """
        Args:
            なし。

        Returns:
            初期化済みのIM920 Client。

        About:
            enterより前の通信操作を検出し、未初期化時は例外を送出する。
        """
        if self.im920 is None:
            msg = "Handler.enter() must be called before use"
            raise RuntimeError(msg)
        return self.im920

    @staticmethod
    def _normalize_axis(value: int, axis_info: AxisInfo | None) -> int:
        """
        Args:
            value: Controller軸のraw入力値。
            axis_info: 軸の最小値と最大値を含む情報。

        Returns:
            deadzone適用後に送信範囲へ正規化した値。

        About:
            Controllerごとの入力範囲を走行Command用の共通範囲へ変換する。
        """
        if axis_info is None or axis_info.maximum == axis_info.minimum:
            return 0
        center = (axis_info.minimum + axis_info.maximum) / 2.0
        half_range = (axis_info.maximum - axis_info.minimum) / 2.0
        normalized = (value - center) / half_range
        if abs(normalized) < STICK_DEADZONE:
            normalized = 0.0
        normalized = max(-1.0, min(1.0, normalized))
        return int(normalized * STICK_SEND_MAX)

    @staticmethod
    def _is_trigger_pressed(state: ControllerState, axis: AxisCode) -> bool:
        """
        Args:
            state: 最新のController状態。
            axis: 判定対象のトリガー軸。

        Returns:
            設定比率以上押されている場合はTrue、それ以外はFalse。

        About:
            トリガー入力をslow modeの有効判定へ変換する。
        """
        axis_info = state.axis_info.get(axis)
        if axis_info is None or axis_info.maximum <= axis_info.minimum:
            return False
        value = state.axis_values.get(axis, axis_info.minimum)
        pressed_ratio = (value - axis_info.minimum) / (axis_info.maximum - axis_info.minimum)
        return pressed_ratio >= TRIGGER_ACTIVE_RATIO

    def _make_drive_command(self, state: ControllerState) -> DriveCommand:
        """
        Args:
            state: 最新のController状態。

        Returns:
            stickとtriggerの状態から生成した走行Command。

        About:
            各軸を正規化し、必要に応じてslow mode係数を適用する。
        """
        lx = self._normalize_axis(
            state.axis_values.get(AxisCode.LEFT_STICK_X, 0),
            state.axis_info.get(AxisCode.LEFT_STICK_X),
        )
        ly = self._normalize_axis(
            state.axis_values.get(AxisCode.LEFT_STICK_Y, 0),
            state.axis_info.get(AxisCode.LEFT_STICK_Y),
        )
        rx = self._normalize_axis(
            state.axis_values.get(AxisCode.RIGHT_STICK_X, 0),
            state.axis_info.get(AxisCode.RIGHT_STICK_X),
        )
        vx = -ly
        vy = lx
        wz = -rx
        slow_mode = self._is_trigger_pressed(
            state,
            AxisCode.LEFT_TRIGGER_L2,
        ) or self._is_trigger_pressed(state, AxisCode.RIGHT_TRIGGER_R2)
        if slow_mode:
            vx = int(vx * SLOW_MODE_MULTIPLIER)
            vy = int(vy * SLOW_MODE_MULTIPLIER)
            wz = int(wz * SLOW_MODE_MULTIPLIER)
        return self.commands.drive(vx, vy, wz)

    def handle_axis(self, event: AxisInputEvent, state: ControllerState) -> None:
        """
        Args:
            event: 受信したController軸イベント。
            state: 更新対象のController状態。

        Returns:
            なし。

        About:
            後続の周期走行Command生成で使用する軸状態を更新する。
        """
        state.axis_values[event.code] = event.value

    def handle_button(self, event: ButtonEvent) -> None:
        """
        Args:
            event: 受信したControllerボタンイベント。

        Returns:
            なし。

        About:
            ボタン入力を停止、出力変更、StepAssist resetへ変換して送信する。
        """
        logger.info(
            "[ROBOT] ボタン入力: button=%s state=%s",
            event.code.display_name,
            event.state.name,
        )
        if event.state is ButtonState.PRESSED:
            command = None
            if event.code is ButtonCode.CROSS_BTN:
                command = self.commands.stop()
            elif event.code is ButtonCode.PS_BTN:
                command = self.commands.emergency_stop()
            elif event.code is ButtonCode.L1_BTN:
                command = self.commands.change_power(-DRIVE_POWER_STEP)
            elif event.code is ButtonCode.R1_BTN:
                command = self.commands.change_power(DRIVE_POWER_STEP)
            elif event.code is ButtonCode.CIRCLE_BTN:
                command = self.commands.reset_step_assist()
            if command is not None:
                self._get_im920().send(command)

    def tick(
        self,
        now: float,
        state: ControllerState,
        last_send: float,
    ) -> float:
        """
        Args:
            now: 現在時刻を秒で表した値。
            state: 最新のController状態。
            last_send: 前回走行Commandを送信した時刻。

        Returns:
            走行Commandを最後に送信した時刻。

        About:
            IM920 responseをpollし、設定周期に達した場合は走行Commandを送る。
        """
        im920 = self._get_im920()
        response = im920.poll()
        if response is not None:
            logger.info("[ROBOT] ESP32から文字列を受信しました: %s", response.text)
            print("ESP32 <-", response.text)  # noqa: T201
        if now - last_send < (1.0 / DRIVE_HZ):
            return last_send
        im920.send(self._make_drive_command(state))
        return now
