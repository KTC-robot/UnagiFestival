import logging
import re
import time

import RPi.GPIO as GPIO

from evdev import ecodes

from unagifestival.tools.ps_controller import im_wireless as imw
from unagifestival.tools.ps_controller.config import (
    DRIVE_HZ,
    DRIVE_POWER_STEP,
    IM920_CMD_MAX_LEN,
    LED_PIN,
    SERVO_BUTTON_ACTIONS,
    SERVO_CHANNEL_COUNT,
    SERVO_ENABLED,
    SERVO_HOME_ANGLE,
    SERVO_MAX_ANGLE,
    SERVO_MIN_ANGLE,
    SERVO_SEND_HOME_ON_START,
    SERVO_TOGGLE_ACTIONS,
    SLAVE_ADR,
    STICK_DEADZONE,
    STICK_SEND_MAX,
    TRIGGER_SEND_MAX,
    TX_LED_PULSE_SEC,
)
from unagifestival.tools.ps_controller.enums import (
    ButtonCode,
    ButtonEvent,
    ButtonState,
)
from unagifestival.tools.ps_controller.robot_api import RobotApi
from unagifestival.tools.ps_controller.transport import Im920Transport

logger = logging.getLogger("teensy_log")



class RobotHandler:
    """PS5コントローラー入力を意味のあるロボット操作へ変換する。"""

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
        self.servo_toggle_state: dict[tuple[ButtonCode, int], bool] = {}

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
        except Exception:
            logger.warning("[ROBOT] LED off failed", exc_info=True)

        try:
            self.transport.cleanup()
        except Exception:
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
    def _normalize_axis(value: int, axis_info) -> int:
        """スティック入力を -127 ～ 127 に変換する。"""
        if axis_info is None:
            return 0

        min_v = axis_info.min
        max_v = axis_info.max

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
    def _normalize_trigger(value: int, axis_info) -> int:
        """L2/R2トリガーを 0 ～ 255 に変換する。"""
        if axis_info is None:
            return 0

        min_v = axis_info.min
        max_v = axis_info.max

        if max_v == min_v:
            return 0

        normalized = (value - min_v) / (max_v - min_v)
        normalized = max(0.0, min(1.0, normalized))

        return int(normalized * TRIGGER_SEND_MAX)

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

        if len(payload) < 2:
            return ""

        return self._hex_to_text(payload)

    def _poll_im920_response(self) -> None:
        try:
            data = self.transport.read()
        except Exception:
            logger.warning("[ROBOT] IM920 read failed", exc_info=True)
            return

        if not data:
            return

        data = data.strip()

        if not data:
            return

        normalized = self._normalize_im920_data(data)
        logger.info("[ROBOT] IM920 <- %r", normalized)

        rx_text = self._parse_im920_rx_text(normalized)

        if rx_text:
            logger.info("[ROBOT] ESP32 TEXT <- %s", rx_text)
            print("ESP32 <-", rx_text)

    # ============================================================
    # Servo configuration / control
    # ============================================================

    @staticmethod
    def _validate_servo_config() -> None:
        """16チャンネル分の設定が揃っているか起動時に確認する。"""
        settings = {
            "SERVO_ENABLED": SERVO_ENABLED,
            "SERVO_MIN_ANGLE": SERVO_MIN_ANGLE,
            "SERVO_MAX_ANGLE": SERVO_MAX_ANGLE,
            "SERVO_HOME_ANGLE": SERVO_HOME_ANGLE,
        }

        for name, values in settings.items():
            if len(values) != SERVO_CHANNEL_COUNT:
                raise ValueError(
                    f"{name} must contain {SERVO_CHANNEL_COUNT} values, "
                    f"but contains {len(values)}"
                )

        for channel in range(SERVO_CHANNEL_COUNT):
            minimum = SERVO_MIN_ANGLE[channel]
            maximum = SERVO_MAX_ANGLE[channel]
            home = SERVO_HOME_ANGLE[channel]

            if not 0 <= minimum <= maximum <= 180:
                raise ValueError(
                    f"Invalid servo angle range: CH{channel} "
                    f"min={minimum} max={maximum}"
                )

            if not minimum <= home <= maximum:
                raise ValueError(
                    f"SERVO_HOME_ANGLE[{channel}]={home} is outside "
                    f"{minimum}..{maximum}"
                )

        valid_buttons = set(ButtonCode)
        drive_reserved_buttons = {
            ButtonCode.CROSS_BTN,
            ButtonCode.L1_BTN,
            ButtonCode.R1_BTN,
            ButtonCode.PS_BTN,
        }

        for button, actions in SERVO_BUTTON_ACTIONS.items():
            if button not in valid_buttons:
                raise ValueError(
                    f"Invalid servo button: {button}"
                )

            if button in drive_reserved_buttons:
                logger.warning(
                    "[SERVO] Button %s is also used by chassis control.",
                    button.display_name,
                )

            for channel, angle in actions:
                RobotHandler._validate_servo_action(
                    button,
                    channel,
                    angle,
                )

        for button, actions in SERVO_TOGGLE_ACTIONS.items():
            if button not in valid_buttons:
                raise ValueError(
                    f"Invalid servo toggle button: {button}"
                )

            if button in drive_reserved_buttons:
                logger.warning(
                    "[SERVO] Button %s is also used by chassis control.",
                    button.display_name,
                )

            for channel, angle_a, angle_b in actions:
                RobotHandler._validate_servo_action(
                    button,
                    channel,
                    angle_a,
                )
                RobotHandler._validate_servo_action(
                    button,
                    channel,
                    angle_b,
                )

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
    def _validate_servo_action(
        button: ButtonCode,
        channel: int,
        angle: int,
    ) -> None:
        """ボタン割り当てのチャンネルと角度を検証する。"""
        if not 0 <= channel < SERVO_CHANNEL_COUNT:
            raise ValueError(
                f"Invalid servo channel: button={button.display_name} "
                f"channel={channel}"
            )

        if not SERVO_ENABLED[channel]:
            raise ValueError(
                f"Servo CH{channel} is assigned to button "
                f"{button.display_name}, "
                "but the channel is disabled in SERVO_ENABLED."
            )

        minimum = SERVO_MIN_ANGLE[channel]
        maximum = SERVO_MAX_ANGLE[channel]

        if not minimum <= angle <= maximum:
            raise ValueError(
                f"Servo angle is outside the configured range: "
                f"button={button.display_name} CH{channel} angle={angle} "
                f"range={minimum}..{maximum}"
            )

    def _send_servo(self, channel: int, angle: int) -> None:
        """PCA9685の指定チャンネルを指定角度へ動かす。"""
        if not 0 <= channel < SERVO_CHANNEL_COUNT:
            logger.warning("[SERVO] Invalid channel: %d", channel)
            return

        if not SERVO_ENABLED[channel]:
            logger.warning(
                "[SERVO] CH%d is disabled. Set SERVO_ENABLED[%d] to True.",
                channel,
                channel,
            )
            return

        angle = max(
            SERVO_MIN_ANGLE[channel],
            min(SERVO_MAX_ANGLE[channel], angle),
        )

        logger.info("[SERVO] SEND CH=%d ANGLE=%d", channel, angle)
        self.robot.set_servo(channel, angle)

    def _send_servo_home_positions(self) -> None:
        """有効な全チャンネルを待機角度へ移動する。"""
        for channel in range(SERVO_CHANNEL_COUNT):
            if SERVO_ENABLED[channel]:
                self._send_servo(channel, SERVO_HOME_ANGLE[channel])
                time.sleep(0.05)

    def _handle_servo_button(self, event: ButtonEvent) -> None:
        """設定されたボタン操作をサーボ命令へ変換する。"""
        if event.state is not ButtonState.PRESSED:
            return

        for channel, angle in SERVO_BUTTON_ACTIONS.get(event.code, ()):
            self._send_servo(channel, angle)

        toggle_actions = SERVO_TOGGLE_ACTIONS.get(event.code, ())

        for action_index, (channel, angle_a, angle_b) in enumerate(
            toggle_actions
        ):
            state_key = (event.code, action_index)
            use_angle_b = self.servo_toggle_state.get(state_key, False)

            angle = angle_b if use_angle_b else angle_a
            self.servo_toggle_state[state_key] = not use_angle_b
            self._send_servo(channel, angle)

    # ============================================================
    # Input to robot action
    # ============================================================

    def _make_drive_command(self, raw: dict, info: dict) -> tuple[int, int, int]:
        lx = self._normalize_axis(
            raw.get(ecodes.ABS_X, 0),
            info.get(ecodes.ABS_X),
        )

        ly = self._normalize_axis(
            raw.get(ecodes.ABS_Y, 0),
            info.get(ecodes.ABS_Y),
        )

        rx = self._normalize_axis(
            raw.get(ecodes.ABS_RX, 0),
            info.get(ecodes.ABS_RX),
        )

        dpad_x = raw.get(ecodes.ABS_HAT0X, 0)
        dpad_y = raw.get(ecodes.ABS_HAT0Y, 0)

        if dpad_x != 0 or dpad_y != 0:
            vx = dpad_y * STICK_SEND_MAX
            vy = dpad_x * STICK_SEND_MAX
            wz = 0
        else:
            vx = ly
            vy = lx
            wz = rx

        logger.info(
            "[ROBOT] DRIVE -> VX=%d VY=%d WZ=%d",
            vx,
            vy,
            wz,
        )

        return vx, vy, wz

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

    # ============================================================
    # Event handlers
    # ============================================================

    def handle_abs(
        self,
        code: int,
        raw: dict,
        info: dict,
    ) -> None:
        self._update_tx_led()

        if code in (ecodes.ABS_HAT0X, ecodes.ABS_HAT0Y):
            vx, vy, wz = self._make_drive_command(raw, info)
            self.robot.drive(vx, vy, wz)

    def handle_key(self, code: int, value: int) -> None:
        self._update_tx_led()

        try:
            state = ButtonState(value)
        except ValueError:
            return

        button = ButtonCode.get_by_code(code)

        if button is None:
            logger.debug(
                "[ROBOT] Unknown KEY code=%d value=%d",
                code,
                value,
            )
            return

        event = ButtonEvent(button, state)
        logger.info(
            "[ROBOT] BUTTON %s state=%s",
            button.display_name,
            state.name,
        )
        self._handle_button_event(event)
        self._handle_servo_button(event)

    def tick(self, now: float, raw: dict, info: dict, last_send: float) -> float:
        self._update_tx_led()
        self._poll_im920_response()

        if now - last_send < (1.0 / DRIVE_HZ):
            return last_send

        vx, vy, wz = self._make_drive_command(raw, info)
        self.robot.drive(vx, vy, wz)

        return now