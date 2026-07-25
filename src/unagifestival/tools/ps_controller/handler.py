import logging
import re
import time

import RPi.GPIO as GPIO
from evdev import ecodes

from unagifestival.tools.ps_controller.config import (
    IM920_CMD_MAX_LEN,
    JOY_HZ,
    LED_PIN,
    SLAVE_ADR,
    STICK_DEADZONE,
    STICK_SEND_MAX,
    TRIGGER_SEND_MAX,
    TX_LED_PULSE_SEC,
)

# src/unagifestival/tools/ps_controller/im_wireless.py を使う
from unagifestival.tools.ps_controller import im_wireless as imw

logger = logging.getLogger("teensy_log")


BUTTON_ID = {
    304: 0,   # CROSS
    305: 1,   # CIRCLE
    307: 2,   # TRIANGLE
    308: 3,   # SQUARE
    310: 4,   # L1
    311: 5,   # R1
    312: 6,   # L2 button
    313: 7,   # R2 button
    314: 8,   # SHARE
    315: 9,   # OPTIONS
    316: 10,  # PS
    317: 11,  # L3
    318: 12,  # R3
    273: 13,  # TOUCHPAD
}


BUTTON_NAME = {
    0: "CROSS",
    1: "CIRCLE",
    2: "TRIANGLE",
    3: "SQUARE",
    4: "L1",
    5: "R1",
    6: "L2_BTN",
    7: "R2_BTN",
    8: "SHARE",
    9: "OPTIONS",
    10: "PS",
    11: "L3",
    12: "R3",
    13: "TOUCHPAD",
}


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
    """

    def __init__(self) -> None:
        logger.info("[ROBOT] IM920-HAT initializing...")

        self.iwc = imw.IMWireClass(SLAVE_ADR)

        GPIO.setup(LED_PIN, GPIO.OUT)
        GPIO.output(LED_PIN, GPIO.LOW)

        self.tx_led_off_at = 0.0

        logger.info("[ROBOT] IM920-HAT initialized")

    def enter(self) -> None:
        logger.info("[ROBOT] PS5 Controller -> IM920-HAT sender start")

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
    def _byte_to_hex(value: int) -> str:
        return f"{value & 0xFF:02X}"

    def _send_hex_payload(self, payload_hex: str, label: str = "") -> None:
        """
        payload_hexをTXDAで送信する。
        """
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
        """
        スティック入力を -127 ～ 127 に変換する。
        """
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
        """
        L2/R2トリガーを 0 ～ 255 に変換する。
        """
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
        """
        IM920-HATから来る文字の上位bit対策。
        """
        return "".join(chr(ord(c) & 0x7F) for c in data)

    @staticmethod
    def _hex_to_text(hex_str: str) -> str:
        try:
            return bytes.fromhex(hex_str).decode("utf-8", errors="replace")
        except ValueError:
            return ""

    def _parse_im920_rx_text(self, line: str) -> str:
        """
        例:
            00,0002,D3:53,54,41,54
        を
            STAT
        に変換する。
        """
        line = line.strip()

        if ":" not in line:
            return ""

        payload = line.split(":", 1)[1]
        payload = re.sub(r"[^0-9A-Fa-f]", "", payload)

        if len(payload) < 2:
            return ""

        return self._hex_to_text(payload)

    def _poll_im920_response(self) -> None:
        """
        IM920からのOK/NGなどの応答確認。
        """
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
    # Packet send
    # ============================================================

    def _send_button(self, button_code: int, state: int) -> None:
        """
        3バイト送信。
        42 = 'B'
        """
        if button_code not in BUTTON_ID:
            return

        button_id = BUTTON_ID[button_code]

        payload = (
            "42"
            + self._byte_to_hex(button_id)
            + self._byte_to_hex(state)
        )

        logger.info(
            "[ROBOT] BUTTON -> %s state=%d",
            BUTTON_NAME.get(button_id, button_id),
            state,
        )

        self._send_hex_payload(payload, "BUTTON")

    def _send_dpad(self, axis: str, value: int) -> None:
        """
        3バイト送信。
        44 = 'D'
        axis_id: 0 = X, 1 = Y
        value: -1, 0, 1
        """
        axis_id = 0 if axis == "X" else 1

        payload = (
            "44"
            + self._byte_to_hex(axis_id)
            + self._byte_to_hex(value)
        )

        logger.info("[ROBOT] DPAD -> %s value=%d", axis, value)

        self._send_hex_payload(payload, "DPAD")

    def _make_joy_payload(self, raw: dict, info: dict) -> str:
        """
        9バイト送信。
        4A = 'J'

        J, LX, LY, RX, RY, L2, R2, DPAD_X, DPAD_Y
        """
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
            "[ROBOT] JOY -> LX=%d LY=%d RX=%d RY=%d L2=%d R2=%d DPX=%d DPY=%d",
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
        """
        スティック・十字キーなどのABSイベント処理。
        十字キーは即時DPADパケットでも送る。
        """
        self._update_tx_led()

        if code == ecodes.ABS_HAT0X:
            self._send_dpad("X", value)

        elif code == ecodes.ABS_HAT0Y:
            self._send_dpad("Y", value)

    def handle_key(self, code: int, value: int) -> None:
        """
        ボタンイベント処理。
        value:
            0 = 離した
            1 = 押した
            2 = 長押し/リピート
        """
        self._update_tx_led()

        if value not in (0, 1):
            return

        self._send_button(code, value)

    def tick(self, now: float, raw: dict, info: dict, last_send: float) -> float:
        """
        周期的にJOYパケットを送信する。
        """
        self._update_tx_led()
        self._poll_im920_response()

        if now - last_send < (1.0 / JOY_HZ):
            return last_send

        joy_payload = self._make_joy_payload(raw, info)
        self._send_hex_payload(joy_payload, "JOY")

        return now