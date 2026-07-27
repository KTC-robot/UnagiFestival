from typing import Final

from unagifestival.tools.ps_controller.enums import ButtonCode

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

# JOYパケット送信周期
JOY_HZ: Final[float] = 5.0

# IM920-HATのI2Cコマンド長制限
IM920_CMD_MAX_LEN: Final[int] = 32

# 送信確認LEDの点灯時間
TX_LED_PULSE_SEC: Final[float] = 0.02


# ============================================================
# コントローラー正規化設定
# ============================================================

STICK_DEADZONE: Final[float] = 0.08
STICK_SEND_MAX: Final[int] = 127
TRIGGER_SEND_MAX: Final[int] = 255


# ============================================================
# PCA9685 / サーボ設定
# ============================================================

SERVO_CHANNEL_COUNT: Final[int] = 16

# CH0、CH1、CH2を使用。
# サーボを追加したら、対応チャンネルをTrueへ変更する。
SERVO_ENABLED: Final[tuple[bool, ...]] = (
    True,  True,  True,  False,  # CH0～CH3
    False, False, False, False,  # CH4～CH7
    False, False, False, False,  # CH8～CH11
    False, False, False, False,  # CH12～CH15
)

# ラズパイ側でも送信角度を安全範囲へ制限する。
SERVO_MIN_ANGLE: Final[tuple[int, ...]] = (
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
)

SERVO_MAX_ANGLE: Final[tuple[int, ...]] = (
    180, 180, 180, 180,
    180, 180, 180, 180,
    180, 180, 180, 180,
    180, 180, 180, 180,
)

SERVO_HOME_ANGLE: Final[tuple[int, ...]] = (
    90, 90, 90, 90,
    90, 90, 90, 90,
    90, 90, 90, 90,
    90, 90, 90, 90,
)

# Trueにすると、ラズパイ側プログラム起動時に
# 有効チャンネルをHOME_ANGLEへ動かす。
# 機構完成前はFalseを推奨。
SERVO_SEND_HOME_ON_START: Final[bool] = False


# ============================================================
# ボタンを押すと指定角度へ動かす
# ============================================================
#
# 形式:
#   button_id: ((channel, angle), ...)
#
# ボタン番号を直接書かず、enums.pyのButtonCodeを使用する。
# 例:
#   ButtonCode.SQUARE_BTN.packet_id
#
# CROSS、L1、R1、PSは足回りで使用するため、
# サーボへ割り当てない。
SERVO_BUTTON_ACTIONS: Final[
    dict[int, tuple[tuple[int, int], ...]]
] = {
    ButtonCode.SQUARE_BTN.packet_id: ((2, 60),),
    ButtonCode.TRIANGLE_BTN.packet_id: ((2, 120),),
    ButtonCode.CIRCLE_BTN.packet_id: ((1, 90),),
    ButtonCode.OPTIONS_BTN.packet_id: ((1, 0),),
}


# ============================================================
# ボタンを押すたびに2つの角度を切り替える
# ============================================================
#
# 形式:
#   button_id: ((channel, angle_a, angle_b), ...)
SERVO_TOGGLE_ACTIONS: Final[
    dict[int, tuple[tuple[int, int, int], ...]]
] = {
    ButtonCode.L2_BTN.packet_id: ((0, 0, 180),),
}