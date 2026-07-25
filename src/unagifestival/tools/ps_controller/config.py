from typing import Final

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

# IM920-HATのI2Cアドレス
SLAVE_ADR: Final[int] = 0x30

# Raspberry Pi側の送信確認LED
LED_PIN: Final[int] = 24

# JOYパケット送信周期
JOY_HZ: Final[float] = 5.0

# IM920-HATのI2Cコマンド長制限対策
IM920_CMD_MAX_LEN: Final[int] = 32

# 送信確認LEDの点灯時間
TX_LED_PULSE_SEC: Final[float] = 0.02


# ============================================================
# コントローラー正規化設定
# ============================================================

# スティックの遊び
STICK_DEADZONE: Final[float] = 0.08

# スティック送信値
STICK_SEND_MAX: Final[int] = 127

# L2/R2トリガー送信値
TRIGGER_SEND_MAX: Final[int] = 255


# ============================================================
# PCA9685 / サーボ設定
# ============================================================

# PCA9685 1台で使用できるチャンネル数
SERVO_CHANNEL_COUNT: Final[int] = 16

# 各チャンネルを使用するかどうか。
# 実際にサーボを接続したチャンネルだけ True に変更する。
SERVO_ENABLED: Final[tuple[bool, ...]] = (
    False, False, False, False,
    False, False, False, False,
    False, False, False, False,
    False, False, False, False,
)

# 各チャンネルの安全な最小角度
SERVO_MIN_ANGLE: Final[tuple[int, ...]] = (
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
)

# 各チャンネルの安全な最大角度
SERVO_MAX_ANGLE: Final[tuple[int, ...]] = (
    180, 180, 180, 180,
    180, 180, 180, 180,
    180, 180, 180, 180,
    180, 180, 180, 180,
)

# 各チャンネルの待機角度
SERVO_HOME_ANGLE: Final[tuple[int, ...]] = (
    90, 90, 90, 90,
    90, 90, 90, 90,
    90, 90, 90, 90,
    90, 90, 90, 90,
)

# True にすると、プログラム開始時に有効チャンネルを HOME_ANGLE へ動かす。
# 機構が未完成の間は False のままを推奨。
SERVO_SEND_HOME_ON_START: Final[bool] = False

# ボタンを押したとき、指定角度へ移動する設定。
# 形式:
#   button_id: ((channel, angle), ...)
#
# 例:
# SERVO_BUTTON_ACTIONS = {
#     3: ((0, 60),),               # SQUARE → CH0を60度
#     2: ((0, 120), (1, 45)),      # TRIANGLE → CH0とCH1を同時動作
# }
SERVO_BUTTON_ACTIONS: Final[dict[int, tuple[tuple[int, int], ...]]] = {
    3: ((2, 60),),    
    2: ((2, 120),),   
    1: ((1, 90),),
    4: ((1,0),),    
}

# ボタンを押すたびに2つの角度を切り替える設定。
# 形式:
#   button_id: ((channel, angle_a, angle_b), ...)
#
# 例:
# SERVO_TOGGLE_ACTIONS = {
#     3: ((0, 60, 120),),          # SQUARE → CH0を60度⇔120度
#     2: ((1, 40, 140),),          # TRIANGLE → CH1を40度⇔140度
# }
SERVO_TOGGLE_ACTIONS: Final[
    dict[int, tuple[tuple[int, int, int], ...]]
] = {
    6: ((0,0,180),),
    
}