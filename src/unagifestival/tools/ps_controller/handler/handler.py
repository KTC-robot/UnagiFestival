import logging
import time

from unagifestival.tools.ps_controller.enum import (
    AxisCode,
    ButtonCode,
    ButtonState,
)
from unagifestival.tools.ps_controller.handler.constants import (
    AIR_TRIGGER_ACTIVE_RATIO,
    DRIVE_HZ,
    STATE_COMMAND_RETRY_COUNT,
    STATE_COMMAND_RETRY_INTERVAL_SECONDS,
    STICK_DEADZONE,
    STICK_SEND_MAX,
)
from unagifestival.tools.ps_controller.handler.validate import (
    validate_handler_config,
)
from unagifestival.tools.ps_controller.im920.client import (
    IM920ClientProtocol,
    create_im920_client,
)
from unagifestival.tools.ps_controller.im920.enum import Md20aState
from unagifestival.tools.ps_controller.im920.factory import CommandFactory
from unagifestival.tools.ps_controller.im920.model import DriveCommand, IM920Command
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
        self._md20a_state = Md20aState.STOPPED
        self._air_trigger_pressed = False

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
            R2入力をAir連射の有効判定へ変換する。
        """
        axis_info = state.axis_info.get(axis)
        if axis_info is None or axis_info.maximum <= axis_info.minimum:
            return False
        value = state.axis_values.get(axis, axis_info.minimum)
        pressed_ratio = (value - axis_info.minimum) / (axis_info.maximum - axis_info.minimum)
        return pressed_ratio >= AIR_TRIGGER_ACTIVE_RATIO

    def _make_drive_command(self, state: ControllerState) -> DriveCommand:
        """
        Args:
            state: 最新のController状態。

        Returns:
            stickの状態から生成した走行Command。

        About:
            各Stick軸を正規化して走行Commandを生成する。
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
        return self.commands.drive(vx, vy, wz)

    def _send_state_command(self, command: IM920Command) -> None:
        """状態変化Commandだけを短い間隔で3回送信する。"""
        im920 = self._get_im920()
        for attempt in range(STATE_COMMAND_RETRY_COUNT):
            im920.send(command)
            if attempt + 1 < STATE_COMMAND_RETRY_COUNT:
                time.sleep(STATE_COMMAND_RETRY_INTERVAL_SECONDS)

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
        previous_value = state.axis_values.get(event.code, 0)
        state.axis_values[event.code] = event.value
        if event.code is AxisCode.DPAD_Y:
            if event.value == previous_value:
                return
            if event.value == -1:
                self._get_im920().send(self.commands.step_assist_toggle_manual_front())
            elif event.value == 1:
                self._get_im920().send(self.commands.step_assist_toggle_manual_rear())
            return
        if event.code is not AxisCode.RIGHT_TRIGGER_R2:
            return
        pressed = self._is_trigger_pressed(state, AxisCode.RIGHT_TRIGGER_R2)
        if pressed == self._air_trigger_pressed:
            return
        self._air_trigger_pressed = pressed
        if pressed:
            logger.info("[AIR] 連射を開始します")
            self._send_state_command(self.commands.air_fire_start())
        else:
            logger.info("[AIR] 連射を停止します")
            self._send_state_command(self.commands.air_fire_stop())

    def handle_button(self, event: ButtonEvent) -> None:
        """
        Args:
            event: 受信したControllerボタンイベント。

        Returns:
            なし。

        About:
            ボタン入力を停止、MD20A状態、StepAssist resetへ変換して送信する。
        """
        logger.info(
            "[ROBOT] ボタン入力: button=%s state=%s",
            event.code.display_name,
            event.state.name,
        )
        if event.state is ButtonState.PRESSED:
            command = None
            if event.code is ButtonCode.CROSS_BTN:
                self._md20a_state = Md20aState.STOPPED
                command = self.commands.stop()
            elif event.code is ButtonCode.PS_BTN:
                command = self.commands.step_assist_toggle_mode()
            elif event.code is ButtonCode.L1_BTN:
                self._md20a_state = (
                    Md20aState.STOPPED if self._md20a_state is Md20aState.REVERSE else Md20aState.REVERSE
                )
                command = self.commands.md20a_set_state(self._md20a_state)
            elif event.code is ButtonCode.R1_BTN:
                self._md20a_state = (
                    Md20aState.STOPPED if self._md20a_state is Md20aState.FORWARD else Md20aState.FORWARD
                )
                command = self.commands.md20a_set_state(self._md20a_state)
            elif event.code is ButtonCode.CIRCLE_BTN:
                command = self.commands.reset_step_assist()
            if command is not None:
                if event.code in (ButtonCode.L1_BTN, ButtonCode.R1_BTN):
                    self._send_state_command(command)
                else:
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
