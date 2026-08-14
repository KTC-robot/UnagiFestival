import logging
import re
import time

from RPi import GPIO

from unagifestival.tools.ps_controller import im_wireless as imw
from unagifestival.tools.ps_controller.config import (
    DRIVE_HZ,
    DRIVE_POWER_STEP,
    IM920_CMD_MAX_LEN,
    LED_PIN,
    SERVO_ALL_CLOSE_ANGLE,
    SERVO_ALL_OPEN_ANGLE,
    SERVO_BUTTON_ACTIONS,
    SERVO_CHANNEL_COUNT,
    SERVO_CHANNELS,
    SERVO_SEND_HOME_ON_START,
    SERVO_TOGGLE_ACTIONS,
    SLAVE_ADR,
    STICK_DEADZONE,
    STICK_SEND_MAX,
    TX_LED_PULSE_SEC,
)
from unagifestival.tools.ps_controller.enums import (
    AxisCode,
    AxisInputEvent,
    ButtonCode,
    ButtonEvent,
    ButtonState,
)
from unagifestival.tools.ps_controller.models import (
    AxisInfo,
    ControllerState,
    DriveCommand,
    ServoAction,
    ServoSetCommand,
    ServoToggleKey,
)
from unagifestival.tools.ps_controller.robot_api import RobotApi
from unagifestival.tools.ps_controller.transport import Im920Transport

logger = logging.getLogger("unagi_log")

MIN_HEX_TEXT_LENGTH = 2
MAX_SERVO_ANGLE = 180
SLOW_MODE_MULTIPLIER = 0.25
TRIGGER_ACTIVE_RATIO = 0.1


class RobotHandler:
    """PS5コントローラー入力を意味のあるロボット操作へ変換する."""

    def __init__(self) -> None:
        logger.info("[ROBOT] IM920-HAT initializing...")

        im920 = imw.IMWireClass(SLAVE_ADR)

        GPIO.setup(LED_PIN, GPIO.OUT)
        GPIO.output(LED_PIN, GPIO.LOW)

        self.tx_led_off_at = 0.0
        self.transport = Im920Transport(
            im920,
            IM920_CMD_MAX_LEN,
            self._pulse_tx_led,
            logger,
        )
        self.robot = RobotApi(self.transport)
        self.servo_toggle_state: dict[ServoToggleKey, bool] = {}

        self._validate_servo_config()

        logger.info("[ROBOT] IM920-HAT initialized")

    def enter(self) -> None:
        logger.info("[ROBOT] PS5 Controller -> IM920-HAT sender start")

        if SERVO_SEND_HOME_ON_START:
            self._send_servo_home_positions()

    def exit(self) -> None:
        logger.info("[ROBOT] 制御終了")

        try:
            GPIO.output(LED_PIN, GPIO.LOW)
        except Exception:  # noqa: BLE001
            logger.warning("[ROBOT] LED off failed", exc_info=True)

        try:
            self.transport.cleanup()
        except Exception:  # noqa: BLE001
            logger.warning("[ROBOT] GPIO cleanup failed", exc_info=True)

    # ============================================================
    # Utility
    # ============================================================

    def _pulse_tx_led(self) -> None:
        GPIO.output(LED_PIN, GPIO.HIGH)
        self.tx_led_off_at = time.time() + TX_LED_PULSE_SEC

    def _update_tx_led(self) -> None:
        if self.tx_led_off_at <= 0:
            return

        if time.time() >= self.tx_led_off_at:
            GPIO.output(LED_PIN, GPIO.LOW)
            self.tx_led_off_at = 0.0

    @staticmethod
    def _normalize_axis(value: int, axis_info: AxisInfo | None) -> int:
        """スティック入力を -127 ~ 127 に変換する."""
        if axis_info is None:
            return 0

        min_v = axis_info.minimum
        max_v = axis_info.maximum

        if max_v == min_v:
            return 0

        center = (min_v + max_v) / 2.0
        half_range = (max_v - min_v) / 2.0

        normalized = (value - center) / half_range

        if abs(normalized) < STICK_DEADZONE:
            normalized = 0.0

        normalized = max(-1.0, min(1.0, normalized))

        return int(normalized * STICK_SEND_MAX)

    @staticmethod
    def _normalize_im920_data(data: str) -> str:
        return "".join(chr(ord(c) & 0x7F) for c in data)

    @staticmethod
    def _hex_to_text(hex_str: str) -> str:
        try:
            return bytes.fromhex(hex_str).decode("utf-8", errors="replace")
        except ValueError:
            return ""

    def _parse_im920_rx_text(self, line: str) -> str:
        line = line.strip()

        if ":" not in line:
            return ""

        payload = line.split(":", 1)[1]
        payload = re.sub(r"[^0-9A-Fa-f]", "", payload)

        if len(payload) < MIN_HEX_TEXT_LENGTH:
            return ""

        return self._hex_to_text(payload)

    def _poll_im920_response(self) -> None:
        try:
            data = self.transport.read()
        except Exception:  # noqa: BLE001
            logger.warning("[ROBOT] IM920 read failed", exc_info=True)
            return

        if not data:
            return

        data = data.strip()

        if not data:
            return

        normalized = self._normalize_im920_data(data)
        logger.debug("[ROBOT] IM920 <- %r", normalized)

        rx_text = self._parse_im920_rx_text(normalized)

        if rx_text:
            logger.info("[ROBOT] ESP32 TEXT <- %s", rx_text)
            print("ESP32 <-", rx_text)  # noqa: T201

    # ============================================================
    # Servo configuration / control
    # ============================================================

    @staticmethod
    def _validate_servo_config() -> None:
        """使用する7チャネル分の設定とボタン操作を起動時に検証する."""
        if len(SERVO_CHANNELS) != SERVO_CHANNEL_COUNT:
            message = (
                f"SERVO_CHANNELS must contain {SERVO_CHANNEL_COUNT} values, "
                f"but contains {len(SERVO_CHANNELS)}"
            )
            raise ValueError(message)

        for channel, config in enumerate(SERVO_CHANNELS):
            if not 0 <= config.min_angle <= config.max_angle <= MAX_SERVO_ANGLE:
                message = (
                    f"Invalid servo angle range: CH{channel} "
                    f"min={config.min_angle} max={config.max_angle}"
                )
                raise ValueError(message)

            if not config.min_angle <= config.home_angle <= config.max_angle:
                message = (
                    f"Servo home angle is outside the range: CH{channel} "
                    f"home={config.home_angle} "
                    f"range={config.min_angle}..{config.max_angle}"
                )
                raise ValueError(message)

        RobotHandler._validate_servo_actions()
        RobotHandler._warn_duplicate_servo_actions()

    @staticmethod
    def _is_trigger_pressed(
        state: ControllerState,
        axis: AxisCode,
    ) -> bool:
        """トリガーが一定量以上押されているか判定する."""
        axis_info = state.axis_info.get(axis)

        if axis_info is None:
            return False

        minimum = axis_info.minimum
        maximum = axis_info.maximum

        if maximum <= minimum:
            return False

        value = state.axis_values.get(axis, minimum)

        pressed_ratio = (value - minimum) / (maximum - minimum)

        return pressed_ratio >= TRIGGER_ACTIVE_RATIO

    @staticmethod
    def _validate_servo_actions() -> None:
        """ボタンに割り当てられた全サーボ操作を検証する."""
        drive_reserved_buttons = {
            ButtonCode.CROSS_BTN,
            ButtonCode.L1_BTN,
            ButtonCode.R1_BTN,
            ButtonCode.PS_BTN,
        }

        for button, actions in SERVO_BUTTON_ACTIONS.items():
            if button in drive_reserved_buttons:
                logger.warning(
                    "[SERVO] Button %s is also used by chassis control.",
                    button.display_name,
                )

            for action in actions:
                RobotHandler._validate_servo_target(
                    button,
                    ServoSetCommand(action.channel, action.angle),
                )

        for button, actions in SERVO_TOGGLE_ACTIONS.items():
            if button in drive_reserved_buttons:
                logger.warning(
                    "[SERVO] Button %s is also used by chassis control.",
                    button.display_name,
                )

            for action in actions:
                RobotHandler._validate_servo_target(
                    button,
                    ServoSetCommand(action.channel, action.angle_a),
                )
                RobotHandler._validate_servo_target(
                    button,
                    ServoSetCommand(action.channel, action.angle_b),
                )

    @staticmethod
    def _warn_duplicate_servo_actions() -> None:
        """直接操作とtoggle操作の重複割り当てを警告する."""
        duplicate_buttons = (
            set(SERVO_BUTTON_ACTIONS)
            & set(SERVO_TOGGLE_ACTIONS)
        )

        for button in sorted(duplicate_buttons, key=lambda item: item.code):
            logger.warning(
                "[SERVO] Button %s has both direct and toggle actions.",
                button.display_name,
            )

    @staticmethod
    def _validate_servo_target(
        button: ButtonCode,
        command: ServoSetCommand,
    ) -> None:
        """ボタン割り当てのチャネルと角度を検証する."""
        if not 0 <= command.channel < SERVO_CHANNEL_COUNT:
            message = (
                f"Invalid servo channel: button={button.display_name} "
                f"channel={command.channel}"
            )
            raise ValueError(message)

        config = SERVO_CHANNELS[command.channel]

        if not config.enabled:
            message = (
                f"Servo CH{command.channel} is assigned to button "
                f"{button.display_name}, "
                "but the channel is disabled in SERVO_CHANNELS."
            )
            raise ValueError(message)

        if not config.min_angle <= command.angle <= config.max_angle:
            message = (
                f"Servo angle is outside the configured range: "
                f"button={button.display_name} CH{command.channel} "
                f"angle={command.angle} "
                f"range={config.min_angle}..{config.max_angle}"
            )
            raise ValueError(message)

    def _send_servo(self, command: ServoSetCommand) -> None:
        """PCA9685の指定チャネルを指定角度へ動かす."""
        if not 0 <= command.channel < SERVO_CHANNEL_COUNT:
            logger.warning("[SERVO] Invalid channel: %d", command.channel)
            return

        config = SERVO_CHANNELS[command.channel]

        if not config.enabled:
            logger.warning(
                "[SERVO] CH%d is disabled in SERVO_CHANNELS.",
                command.channel,
            )
            return

        clamped_command = ServoSetCommand(
            channel=command.channel,
            angle=max(
                config.min_angle,
                min(config.max_angle, command.angle),
            ),
        )

        logger.info(
            "[SERVO] SEND CH=%d ANGLE=%d",
            clamped_command.channel,
            clamped_command.angle,
        )
        self.robot.set_servo(clamped_command)

    def _send_servo_home_positions(self) -> None:
        """有効な全チャネルを待機角度へ移動する."""
        for channel, config in enumerate(SERVO_CHANNELS):
            if config.enabled:
                self._send_servo(ServoSetCommand(channel, config.home_angle))
                time.sleep(0.05)

    def _send_all_servos(self, angle: int) -> None:
        """有効な全サーボを同じ角度へ動かす."""
        for channel, config in enumerate(SERVO_CHANNELS):
            if config.enabled:
                self._send_servo(ServoSetCommand(channel=channel, angle=angle))

    def _handle_servo_button(self, event: ButtonEvent) -> None:
        """設定されたボタン操作をサーボ命令へ変換する."""
        if event.state is not ButtonState.PRESSED:
            return

        for action in SERVO_BUTTON_ACTIONS.get(event.code, ()):
            self._send_servo(self._servo_command(action))

        toggle_actions = SERVO_TOGGLE_ACTIONS.get(event.code, ())

        for action_index, action in enumerate(toggle_actions):
            state_key = ServoToggleKey(event.code, action_index)
            use_angle_b = self.servo_toggle_state.get(state_key, False)

            angle = action.angle_b if use_angle_b else action.angle_a
            self.servo_toggle_state[state_key] = not use_angle_b
            self._send_servo(ServoSetCommand(action.channel, angle))

    @staticmethod
    def _servo_command(action: ServoAction) -> ServoSetCommand:
        return ServoSetCommand(action.channel, action.angle)

    # ============================================================
    # Input to robot action
    # ============================================================

    def _make_drive_command(self, state: ControllerState) -> DriveCommand:
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

        # 走行commandはスティックだけから生成し、DPADの状態は参照しない。
        vx = -ly
        vy = lx
        wz = -rx

        # L2またはR2を押している間は減速モードにする。
        slow_mode = (
            self._is_trigger_pressed(
                state,
                AxisCode.LEFT_TRIGGER_L2,
            )
            or self._is_trigger_pressed(
                state,
                AxisCode.RIGHT_TRIGGER_R2,
            )
        )

        if slow_mode:
            vx = int(vx * SLOW_MODE_MULTIPLIER)
            vy = int(vy * SLOW_MODE_MULTIPLIER)
            wz = int(wz * SLOW_MODE_MULTIPLIER)

        logger.info(
            "[ROBOT] DRIVE -> VX=%d VY=%d WZ=%d SLOW=%s",
            vx,
            vy,
            wz,
            slow_mode,
        )

        return DriveCommand(vx=vx, vy=vy, wz=wz)

    def _handle_button_event(self, event: ButtonEvent) -> None:
        if event.state is not ButtonState.PRESSED:
            return

        if event.code is ButtonCode.CROSS_BTN:
            self.robot.stop()
        elif event.code is ButtonCode.PS_BTN:
            self.robot.emergency_stop()
        elif event.code is ButtonCode.L1_BTN:
            self.robot.change_power(-DRIVE_POWER_STEP)
        elif event.code is ButtonCode.R1_BTN:
            self.robot.change_power(DRIVE_POWER_STEP)
        elif event.code is ButtonCode.CIRCLE_BTN:
            # Circleは走行停止を伴わない段差制御リセット専用ボタンとする。
            self.robot.reset_step_assist()

    # ============================================================
    # Event handlers
    # ============================================================

    def handle_axis(
        self,
        event: AxisInputEvent,
        state: ControllerState,
    ) -> None:
        self._update_tx_led()
        state.axis_values[event.code] = event.value

        if event.code is AxisCode.DPAD_Y:
            if event.value == -1:
                self._send_all_servos(SERVO_ALL_OPEN_ANGLE)
            elif event.value == 1:
                self._send_all_servos(SERVO_ALL_CLOSE_ANGLE)
            return

    def handle_button(self, event: ButtonEvent) -> None:
        self._update_tx_led()
        logger.info(
            "[ROBOT] BUTTON %s state=%s",
            event.code.display_name,
            event.state.name,
        )
        self._handle_button_event(event)
        self._handle_servo_button(event)

    def tick(
        self,
        now: float,
        state: ControllerState,
        last_send: float,
    ) -> float:
        self._update_tx_led()
        self._poll_im920_response()

        if now - last_send < (1.0 / DRIVE_HZ):
            return last_send

        self.robot.drive(self._make_drive_command(state))

        return now
