import logging
import time

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
from unagifestival.tools.ps_controller.servo.mapper import ServoMapper

logger = logging.getLogger("unagi_log")


class Handler:
    """Controller入力から意味Command生成・IM920送受信までを制御する."""

    def __init__(
        self,
        im920: IM920ClientProtocol | None = None,
        servo_mapper: ServoMapper | None = None,
    ) -> None:
        validate_handler_config()
        self.im920 = im920
        self.commands = CommandFactory()
        self.servo_mapper = servo_mapper or ServoMapper()

    def enter(self) -> None:
        """Handlerを開始し、設定されていればサーボhome指令を送る."""
        if self.im920 is None:
            self.im920 = create_im920_client(logger=logger)

        im920 = self._get_im920()
        logger.info("[ROBOT] PS5 Controller -> IM920-HAT sender start")
        commands = self.servo_mapper.startup_commands()
        for index, command in enumerate(commands):
            im920.send(command)
            if index < len(commands) - 1:
                time.sleep(self.servo_mapper.startup_interval_seconds)

    def exit(self) -> None:
        """IM920-HATのresourceを解放する."""
        logger.info("[ROBOT] 制御終了")
        if self.im920 is None:
            return
        try:
            self.im920.close()
        except Exception:  # noqa: BLE001
            logger.warning("[ROBOT] IM920 cleanup failed", exc_info=True)

    def _get_im920(self) -> IM920ClientProtocol:
        """初期化済みのIM920 Clientを返す."""
        if self.im920 is None:
            msg = "Handler.enter() must be called before use"
            raise RuntimeError(msg)
        return self.im920

    @staticmethod
    def _normalize_axis(value: int, axis_info: AxisInfo | None) -> int:
        """スティック入力を-127〜127へ正規化する."""
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
        """指定triggerが設定比率以上押されているか返す."""
        axis_info = state.axis_info.get(axis)
        if axis_info is None or axis_info.maximum <= axis_info.minimum:
            return False
        value = state.axis_values.get(axis, axis_info.minimum)
        pressed_ratio = (value - axis_info.minimum) / (
            axis_info.maximum - axis_info.minimum
        )
        return pressed_ratio >= TRIGGER_ACTIVE_RATIO

    def _make_drive_command(self, state: ControllerState) -> DriveCommand:
        """現在のstick/trigger状態から走行Commandを生成する."""
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
        """軸状態を更新し、DPAD上下を全サーボ操作へ変換する."""
        state.axis_values[event.code] = event.value
        if event.code is not AxisCode.DPAD_Y:
            return
        if event.value == -1:
            command = self.servo_mapper.open_all()
            logger.info("[SERVO] SEND ALL CH0-6 ANGLE=%d", command.angle)
            self._get_im920().send(command)
        elif event.value == 1:
            command = self.servo_mapper.close_all()
            logger.info("[SERVO] SEND ALL CH0-6 ANGLE=%d", command.angle)
            self._get_im920().send(command)

    def handle_button(self, event: ButtonEvent) -> None:
        """ボタンを足回り操作またはサーボCommandへ変換して送信する."""
        logger.info(
            "[ROBOT] BUTTON %s state=%s",
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

        for servo_command in self.servo_mapper.map_button(event):
            logger.info(
                "[SERVO] SEND CH=%d ANGLE=%d",
                servo_command.channel,
                servo_command.angle,
            )
            self._get_im920().send(servo_command)

    def tick(
        self,
        now: float,
        state: ControllerState,
        last_send: float,
    ) -> float:
        """responseをpollし、設定周期で最新走行Commandを送信する."""
        im920 = self._get_im920()
        response = im920.poll()
        if response is not None:
            logger.info("[ROBOT] ESP32 TEXT <- %s", response.text)
            print("ESP32 <-", response.text)  # noqa: T201
        if now - last_send < (1.0 / DRIVE_HZ):
            return last_send
        im920.send(self._make_drive_command(state))
        return now
