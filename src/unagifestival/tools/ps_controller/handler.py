import logging

from unagifestival.tools.ps_controller.enums import AxisCode, ButtonCode
from unagifestival.tools.ps_controller.handler_config import (
    KEEPALIVE_HZ,
    LIN_RPM_DEFAULT,
    MAX_RPM_TEENSY,
    ROT_RPM_DEFAULT,
    SLOW_MODE_FACTOR,
    STEP_MS,
    STICK_DEAD,
    YAGURA_HOME_ANGLE,
    YAGURA_OPEN_ANGLE,
)
from unagifestival.tools.ps_controller.utils import (
    apply_deadband,
    clamp_value,
    normalize_absolute_axis,
)

logger = logging.getLogger("teensy_log")


class RobotHandler:
    def __init__(self) -> None:
        self.lin_rpm = LIN_RPM_DEFAULT
        self.rot_rpm = ROT_RPM_DEFAULT
        self.yagura_open = {3: False, 4: False, 5: False}
        self.up_step = False
        self.victory = False
        self.slow_mode = False
        logger.debug("[ROBOT] RobotHandler initialized")

    def enter(
        self,
    ) -> None:
        logger.debug("[ROBOT] enter called")
        logger.info("[ROBOT] 制御開始")

    def exit(
        self,
    ) -> None:
        logger.debug("[ROBOT] exit called")
        logger.info("[ROBOT] 制御終了 (STOP)")

    def _toggle_yagura(self, ch: int) -> None:
        self.yagura_open[ch] = not self.yagura_open[ch]
        is_open = self.yagura_open[ch]
        sv_val = YAGURA_HOME_ANGLE if is_open else YAGURA_OPEN_ANGLE
        logger.debug("[ROBOT] _toggle_yagura: CH%s, is_open=%s, sv_val=%d", ch, is_open, sv_val)
        logger.info("[ROBOT] CH%d %s", ch, ("Open" if is_open else "Close"))

    def handle_abs(self, code: int, value: int) -> None:
        logger.debug("[ROBOT] handle_abs: code=%d, value=%d", code, value)
        if code == AxisCode.DPAD_X.code:
            if value == 1:
                self._toggle_yagura(3)
            elif value == -1:
                self._toggle_yagura(5)
        elif code == AxisCode.DPAD_Y.code:
            if value == -1:
                self._toggle_yagura(4)
            elif value == 1:
                self.up_step = not self.up_step
                cmd = f"STP DOWN {STEP_MS}" if self.up_step else f"STP UP {STEP_MS}"
                logger.debug("[ROBOT] Step command: %s", cmd)
                logger.info("[ROBOT] STEP %s", "UP" if self.up_step else "DOWN")

    def handle_key(self, code: int, value: int) -> None:
        logger.debug("[ROBOT] handle_key: code=%d, value=%d", code, value)
        if code in (ButtonCode.L2_BTN.code, ButtonCode.R2_BTN.code):
            self.slow_mode = value == 1
            logger.debug("[ROBOT] Slow mode: %s", self.slow_mode)
            return

        if value != 1:
            return

        if code == ButtonCode.PS_BTN.code:
            self.victory = not self.victory
            cmd = "WIN HOME" if self.victory else "WIN"
            logger.debug("[ROBOT] Victory command: %s", cmd)
            logger.info("[ROBOT] WIN FLAG %s", "UP" if self.victory else "DOWN")

    def tick(self, now: float, raw: dict, info: dict, last_send: float) -> float:
        if now - last_send < (1.0 / KEEPALIVE_HZ):
            return last_send

        lx = normalize_absolute_axis(raw.get(AxisCode.LEFT_STICK_X.code, 0), info.get(AxisCode.LEFT_STICK_X.code))
        ly = normalize_absolute_axis(raw.get(AxisCode.LEFT_STICK_Y.code, 0), info.get(AxisCode.LEFT_STICK_Y.code))
        rx = normalize_absolute_axis(
            raw.get(AxisCode.RIGHT_STICK_X.code, 0),
            info.get(AxisCode.RIGHT_STICK_X.code),
        )

        vx = apply_deadband(ly, STICK_DEAD)
        vy = apply_deadband(lx, STICK_DEAD)
        wz = apply_deadband(rx, STICK_DEAD)

        sx = clamp_value(self.lin_rpm / MAX_RPM_TEENSY, 0.0, 1.0)
        sz = clamp_value(self.rot_rpm / MAX_RPM_TEENSY, 0.0, 1.0)

        if self.slow_mode:
            sx *= SLOW_MODE_FACTOR
            sz *= SLOW_MODE_FACTOR

        chas_cmd = f"CHAS {vx * sx:.3f} {vy * sx:.3f} {wz * sz:.3f}"
        logger.info(
            "[ROBOT] tick: lx=%.2f, ly=%.2f, rx=%.2f | vx=%.2f, vy=%.2f, wz=%.2f | cmd='%s'",
            lx,
            ly,
            rx,
            vx,
            vy,
            wz,
            chas_cmd,
        )
        return now
