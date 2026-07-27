import logging
import re
import time

import RPi.GPIO as GPIO
from evdev import ecodes

from unagifestival.tools.ps_controller.config import (
    IM920_CMD_MAX_LEN,
    JOY_HZ,
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

from unagifestival.tools.ps_controller import im_wireless as imw
from unagifestival.tools.ps_controller.enums import ButtonCode

logger = logging.getLogger("teensy_log")



class RobotHandler:
    """
    PS5コントローラー入力をIM920-HATで送信する。

    送信パケット:
        BUTTON:
            42 button_id state

        DPAD:
            44 axis_id value

        JOY:
            4A LX LY RX RY L2 R2 DPAD_X DPAD_Y

        SERVO:
            53 channel angle
    """

    def __init__(self) -> None:
        logger.info("[ROBOT] IM920-HAT initializing...")

        self.iwc = imw.IMWireClass(SLAVE_ADR)

        GPIO.setup(LED_PIN, GPIO.OUT)
        GPIO.output(LED_PIN, GPIO.LOW)

        self.tx_led_off_at = 0.0
        self.servo_toggle_state: dict[tuple[int, int], bool] = {}

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
            self.iwc.gpio_clean()
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
    def _button_name(button_id: int) -> str:
        button = ButtonCode.get_by_packet_id(button_id)

        if button is None:
            return f"UNKNOWN({button_id})"

        return button.display_name

    @staticmethod
    def _byte_to_hex(value: int) -> str:
        return f"{value & 0xFF:02X}"

    def _send_hex_payload(self, payload_hex: str, label: str = "") -> None:
        """payload_hexをTXDAで送信する。"""
        cmd = "TXDA " + payload_hex

        if len(cmd) > IM920_CMD_MAX_LEN:
            logger.warning("[ROBOT] SKIP command too long: %s", cmd)
            return

        logger.info("[ROBOT] SEND %s -> %s", label, cmd)

        try:
            self.iwc.Write_920(cmd)
            self._pulse_tx_led()
        except Exception:
            logger.warning("[ROBOT] IM920 send failed: %s", cmd, exc_info=True)

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
            data = self.iwc.Read_920()
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

        valid_button_ids = ButtonCode.valid_packet_ids()
        drive_reserved_button_ids = {
            ButtonCode.CROSS_BTN.packet_id,
            ButtonCode.L1_BTN.packet_id,
            ButtonCode.R1_BTN.packet_id,
            ButtonCode.PS_BTN.packet_id,
        }

        for button_id, actions in SERVO_BUTTON_ACTIONS.items():
            if button_id not in valid_button_ids:
                raise ValueError(
                    f"Invalid servo button ID: {button_id}"
                )

            if button_id in drive_reserved_button_ids:
                logger.warning(
                    "[SERVO] Button %s is also used by chassis control.",
                    RobotHandler._button_name(button_id),
                )

            for channel, angle in actions:
                RobotHandler._validate_servo_action(
                    button_id,
                    channel,
                    angle,
                )

        for button_id, actions in SERVO_TOGGLE_ACTIONS.items():
            if button_id not in valid_button_ids:
                raise ValueError(
                    f"Invalid servo toggle button ID: {button_id}"
                )

            if button_id in drive_reserved_button_ids:
                logger.warning(
                    "[SERVO] Button %s is also used by chassis control.",
                    RobotHandler._button_name(button_id),
                )

            for channel, angle_a, angle_b in actions:
                RobotHandler._validate_servo_action(
                    button_id,
                    channel,
                    angle_a,
                )
                RobotHandler._validate_servo_action(
                    button_id,
                    channel,
                    angle_b,
                )

        duplicate_buttons = (
            set(SERVO_BUTTON_ACTIONS)
            & set(SERVO_TOGGLE_ACTIONS)
        )

        for button_id in sorted(duplicate_buttons):
            logger.warning(
                "[SERVO] Button %s has both direct and toggle actions.",
                RobotHandler._button_name(button_id),
            )

    @staticmethod
    def _validate_servo_action(
        button_id: int,
        channel: int,
        angle: int,
    ) -> None:
        """ボタン割り当てのチャンネルと角度を検証する。"""
        if not 0 <= channel < SERVO_CHANNEL_COUNT:
            raise ValueError(
                f"Invalid servo channel: button={button_id} "
                f"channel={channel}"
            )

        if not SERVO_ENABLED[channel]:
            raise ValueError(
                f"Servo CH{channel} is assigned to button {button_id}, "
                "but the channel is disabled in SERVO_ENABLED."
            )

        minimum = SERVO_MIN_ANGLE[channel]
        maximum = SERVO_MAX_ANGLE[channel]

        if not minimum <= angle <= maximum:
            raise ValueError(
                f"Servo angle is outside the configured range: "
                f"button={button_id} CH{channel} angle={angle} "
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

        payload = (
            "53"
            + self._byte_to_hex(channel)
            + self._byte_to_hex(angle)
        )

        logger.info("[SERVO] SEND CH=%d ANGLE=%d", channel, angle)
        self._send_hex_payload(payload, "SERVO")

    def _send_servo_home_positions(self) -> None:
        """有効な全チャンネルを待機角度へ移動する。"""
        for channel in range(SERVO_CHANNEL_COUNT):
            if SERVO_ENABLED[channel]:
                self._send_servo(channel, SERVO_HOME_ANGLE[channel])
                time.sleep(0.05)

    def _handle_servo_button(self, button_id: int, state: int) -> None:
        """設定されたボタン操作をサーボ命令へ変換する。"""
        if state != 1:
            return

        for channel, angle in SERVO_BUTTON_ACTIONS.get(button_id, ()):
            self._send_servo(channel, angle)

        toggle_actions = SERVO_TOGGLE_ACTIONS.get(button_id, ())

        for action_index, (channel, angle_a, angle_b) in enumerate(
            toggle_actions
        ):
            state_key = (button_id, action_index)
            use_angle_b = self.servo_toggle_state.get(state_key, False)

            angle = angle_b if use_angle_b else angle_a
            self.servo_toggle_state[state_key] = not use_angle_b
            self._send_servo(channel, angle)

    # ============================================================
    # Packet send
    # ============================================================

    def _send_button(
        self,
        button: ButtonCode,
        state: int,
    ) -> None:
        button_id = button.packet_id

        payload = (
            "42"
            + self._byte_to_hex(button_id)
            + self._byte_to_hex(state)
        )

        logger.info(
            "[ROBOT] BUTTON -> %s state=%d",
            button.display_name,
            state,
        )

        self._send_hex_payload(payload, "BUTTON")

    def _send_dpad(self, axis: str, value: int) -> None:
        axis_id = 0 if axis == "X" else 1

        payload = (
            "44"
            + self._byte_to_hex(axis_id)
            + self._byte_to_hex(value)
        )

        logger.info("[ROBOT] DPAD -> %s value=%d", axis, value)

        self._send_hex_payload(payload, "DPAD")

    def _make_joy_payload(self, raw: dict, info: dict) -> str:
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

        ry = self._normalize_axis(
            raw.get(ecodes.ABS_RY, 0),
            info.get(ecodes.ABS_RY),
        )

        l2_code = getattr(ecodes, "ABS_Z", 2)
        r2_code = getattr(ecodes, "ABS_RZ", 5)

        l2 = self._normalize_trigger(
            raw.get(l2_code, 0),
            info.get(l2_code),
        )

        r2 = self._normalize_trigger(
            raw.get(r2_code, 0),
            info.get(r2_code),
        )

        dpad_x = raw.get(ecodes.ABS_HAT0X, 0)
        dpad_y = raw.get(ecodes.ABS_HAT0Y, 0)

        payload = (
            "4A"
            + self._byte_to_hex(lx)
            + self._byte_to_hex(ly)
            + self._byte_to_hex(rx)
            + self._byte_to_hex(ry)
            + self._byte_to_hex(l2)
            + self._byte_to_hex(r2)
            + self._byte_to_hex(dpad_x)
            + self._byte_to_hex(dpad_y)
        )

        logger.info(
            "[ROBOT] JOY -> LX=%d LY=%d RX=%d RY=%d "
            "L2=%d R2=%d DPX=%d DPY=%d",
            lx,
            ly,
            rx,
            ry,
            l2,
            r2,
            dpad_x,
            dpad_y,
        )

        return payload

    # ============================================================
    # Event handlers
    # ============================================================

    def handle_abs(self, code: int, value: int) -> None:
        self._update_tx_led()

        if code == ecodes.ABS_HAT0X:
            self._send_dpad("X", value)

        elif code == ecodes.ABS_HAT0Y:
            self._send_dpad("Y", value)

    def handle_key(self, code: int, value: int) -> None:
        self._update_tx_led()

        if value not in (0, 1):
            return

        button = ButtonCode.get_by_code(code)

        if button is None:
            logger.debug(
                "[ROBOT] Unknown KEY code=%d value=%d",
                code,
                value,
            )
            return

        self._send_button(button, value)
        self._handle_servo_button(button.packet_id, value)

    def tick(self, now: float, raw: dict, info: dict, last_send: float) -> float:
        self._update_tx_led()
        self._poll_im920_response()

        if now - last_send < (1.0 / JOY_HZ):
            return last_send

        joy_payload = self._make_joy_payload(raw, info)
        self._send_hex_payload(joy_payload, "JOY")

        return now