"""GPIO/I2CでIM920-HATを制御する低レイヤdriver."""

import contextlib
import time

import smbus2 as smbus

from RPi import GPIO

from unagifestival.tools.ps_controller.im920.constants import (
    BUSY_PIN,
    IRQ_PIN,
    RESET_PIN,
    RX_BUFFER_SIZE,
    RX_DRAIN_LIMIT,
    SLEEP_PIN,
    XMIT_PIN,
)


class IM920HatDriver:
    """IM920-HATをI2C経由で読み書きする."""

    def __init__(self, slave_address: int) -> None:
        self._head = 0
        self._tail = 0
        self._count = 0
        self._buffer: list[str] = [""] * RX_BUFFER_SIZE
        self._slave_address = slave_address

        GPIO.setwarnings(False)  # noqa: FBT003
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RESET_PIN, GPIO.OUT)
        GPIO.setup(XMIT_PIN, GPIO.IN)
        GPIO.setup(SLEEP_PIN, GPIO.IN)
        GPIO.setup(IRQ_PIN, GPIO.IN)
        GPIO.setup(BUSY_PIN, GPIO.IN)
        GPIO.output(RESET_PIN, 1)

        self._i2c: smbus.SMBus = smbus.SMBus(1)
        for _ in range(300):
            with contextlib.suppress(OSError):
                self._i2c.read_byte(self._slave_address)
        self._reboot()

    def _reboot(self) -> None:
        GPIO.output(RESET_PIN, 0)
        time.sleep(0.5)
        GPIO.output(RESET_PIN, 1)
        time.sleep(0.5)

    def write(self, command: str) -> None:
        """IM920-HATへcommand文字列を送信する."""
        if not command:
            return
        if command[0] != "?":
            while GPIO.input(BUSY_PIN):
                time.sleep(0.001)
        self._i2c.write_i2c_block_data(
            self._slave_address,
            0,
            [ord(character) for character in command],
        )

    def read(self) -> str:
        """IM920-HATから受信済みframeを最大1件返す."""
        if GPIO.input(IRQ_PIN) == GPIO.HIGH:
            for _ in range(RX_DRAIN_LIMIT):
                if GPIO.input(IRQ_PIN) != GPIO.HIGH or not self._read_from_i2c():
                    break
        if self._count < 1:
            return ""
        frame = self._buffer[self._head]
        self._head = (self._head + 1) & (RX_BUFFER_SIZE - 1)
        self._count -= 1
        return frame

    def _read_from_i2c(self) -> bool:
        try:
            received_length = self._i2c.read_byte(self._slave_address)
        except OSError:
            return False
        if received_length < 1:
            return False

        frame = ""
        while received_length >= 1:
            try:
                frame += chr(self._i2c.read_byte(self._slave_address))
            except OSError:
                return False
            received_length -= 1
        if not frame:
            return False

        self._buffer[self._tail] = frame
        self._tail = (self._tail + 1) & (RX_BUFFER_SIZE - 1)
        self._count += 1
        if self._count > RX_BUFFER_SIZE:
            self._count = RX_BUFFER_SIZE
            self._head = (self._head + 1) & (RX_BUFFER_SIZE - 1)
        return True

    def close(self) -> None:
        """GPIO resourceを解放する."""
        GPIO.cleanup()
