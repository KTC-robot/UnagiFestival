from typing import Final

# ===== Rates & Constants =====
KEEPALIVE_HZ: Final[float] = 50.0
MAX_RPM_TEENSY: Final[int] = 8000
LIN_RPM_DEFAULT: Final[int] = 8000
ROT_RPM_DEFAULT: Final[int] = 8000

# ===== DRIVE Settings =====
STICK_DEAD: Final[float] = 0.08  # スティックのデッドゾーン

# ===== Handler Control Settings =====
SLOW_MODE_FACTOR: Final[float] = 0.35  # スローモードの速度係数
STEP_MS: Final[int] = 750  # ステップモーターの動作秒数
YAGURA_HOME_ANGLE: Final[int] = 90  # ヤグラアームの初期角度
YAGURA_OPEN_ANGLE: Final[int] = 60  # ヤグラアームの開き角度
