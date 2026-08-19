from typing import Final

SLAVE_ADDRESS: Final[int] = 0x30
IM920_COMMAND_MAX_LENGTH: Final[int] = 32
RX_BUFFER_SIZE: Final[int] = 0x400
RX_DRAIN_LIMIT: Final[int] = 64

# IM920-HAT GPIO pin assignment (BCM)
IRQ_PIN: Final[int] = 17
XMIT_PIN: Final[int] = 18
SLEEP_PIN: Final[int] = 22
RESET_PIN: Final[int] = 23
BUSY_PIN: Final[int] = 27

GAIN_WIRE_SCALE: Final[int] = 1000
GAIN_TUNING_DURATION_UNIT_MS: Final[int] = 100
GAIN_TUNING_MAX_DURATION_MS: Final[int] = 10_000
GAIN_TUNING_DONE_ACK_INDEX: Final[int] = 4
WHEEL_INDEX_MIN: Final[int] = 0
WHEEL_INDEX_MAX: Final[int] = 3
GAIN_DIRECTION_MIN: Final[int] = 0
GAIN_DIRECTION_MAX: Final[int] = 3
WHEEL_GAIN_MIN: Final[float] = 0.50
WHEEL_GAIN_MAX: Final[float] = 1.50
UINT16_MAX_VALUE: Final[int] = 0xFFFF

INVALID_WHEEL_MESSAGE: Final[str] = "wheel must be between 0 and 3"
INVALID_DIRECTION_MESSAGE: Final[str] = "direction must be between 0 and 3"
INVALID_GAIN_MESSAGE: Final[str] = "gain must be between 0.50 and 1.50"
INVALID_TUNING_DURATION_MESSAGE: Final[str] = (
    "duration_ms must be 100..10000 in 100 ms units"
)
INVALID_RESULT_ACK_MESSAGE: Final[str] = "result_index must be between 0 and 4"
