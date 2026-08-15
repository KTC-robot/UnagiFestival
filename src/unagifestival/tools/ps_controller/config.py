from typing import Final

from unagifestival.tools.ps_controller.enums import ButtonCode
from unagifestival.tools.ps_controller.models import (
    ServoAction,
    ServoActionMap,
    ServoChannelConfig,
    ServoToggleAction,
    ServoToggleActionMap,
)

# ============================================================
# 既存設定
# ============================================================

# 互換性のため残す。今回のIM920-HAT方式では基本使わない。
PORT: Final[str] = "/dev/serial0"

AXIS_NORMALIZED_MIN: Final[float] = -1.0
AXIS_NORMALIZED_MAX: Final[float] = 1.0
AXIS_NORMALIZED_CENTER: Final[float] = 0.0


# ============================================================
# IM920-HAT 設定
# ============================================================

SLAVE_ADR: Final[int] = 0x30
LED_PIN: Final[int] = 24

# DRIVEコマンド送信周期
DRIVE_HZ: Final[float] = 5.0
DRIVE_POWER_STEP: Final[int] = 5

# IM920-HATのI2Cコマンド長制限
IM920_CMD_MAX_LEN: Final[int] = 32

# 送信確認LEDの点灯時間
TX_LED_PULSE_SEC: Final[float] = 0.02


# ============================================================
# コントローラー正規化設定
# ============================================================

STICK_DEADZONE: Final[float] = 0.08
STICK_SEND_MAX: Final[int] = 127


# ============================================================
# PCA9685 / サーボ設定
# ============================================================

SERVO_CHANNEL_COUNT: Final[int] = 7
SERVO_ALL_OPEN_ANGLE: Final[int] = 180
SERVO_ALL_CLOSE_ANGLE: Final[int] = 0

SERVO_CHANNELS: Final[tuple[ServoChannelConfig, ...]] = (
    ServoChannelConfig(enabled=True, min_angle=0, max_angle=180, home_angle=90),
    ServoChannelConfig(enabled=True, min_angle=0, max_angle=180, home_angle=90),
    ServoChannelConfig(enabled=True, min_angle=0, max_angle=180, home_angle=90),
    ServoChannelConfig(enabled=True, min_angle=0, max_angle=180, home_angle=90),
    ServoChannelConfig(enabled=True, min_angle=0, max_angle=180, home_angle=90),
    ServoChannelConfig(enabled=True, min_angle=0, max_angle=180, home_angle=90),
    ServoChannelConfig(enabled=True, min_angle=0, max_angle=180, home_angle=90),
)

# Trueにすると、ラズパイ側プログラム起動時に
# 有効チャンネルをHOME_ANGLEへ動かす。
# 機構完成前はFalseを推奨。
SERVO_SEND_HOME_ON_START: Final[bool] = False


# ============================================================
# ボタンを押すと指定角度へ動かす
# ============================================================
# CROSS、CIRCLE、L1、R1、PSは足回りで使用するため、
# サーボへ割り当てない。
SERVO_BUTTON_ACTIONS: Final[ServoActionMap] = {
    ButtonCode.SQUARE_BTN: (ServoAction(channel=2, angle=60),),
    ButtonCode.TRIANGLE_BTN: (ServoAction(channel=2, angle=120),),
    ButtonCode.OPTIONS_BTN: (ServoAction(channel=1, angle=0),),
}


# ============================================================
# ボタンを押すたびに2つの角度を切り替える
# ============================================================
SERVO_TOGGLE_ACTIONS: Final[ServoToggleActionMap] = {
    ButtonCode.L2_BTN: (
        ServoToggleAction(channel=0, angle_a=0, angle_b=180),
    ),
}
