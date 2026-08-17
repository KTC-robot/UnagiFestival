"""
GPIOとI2CでIM920-HATを制御する低レイヤdriverを提供する。

送受信、再起動、受信buffer管理、hardware resource解放を担当する。
"""

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
    """
    Properties:
        なし。
    About:
        GPIOとI2Cを利用してIM920-HATのcommand送信とframe受信を行う。
    """

    def __init__(self, slave_address: int) -> None:
        """
        Args:
            slave_address: IM920-HATのI2C slave address。
        Returns:
            なし。
        About:
            GPIO、I2C、受信bufferを初期化し、IM920-HATを再起動する。
        """
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
        """
        Args:
            なし。
        Returns:
            なし。
        About:
            reset端子を操作してIM920-HATを再起動する。
        """
        GPIO.output(RESET_PIN, 0)
        time.sleep(0.5)
        GPIO.output(RESET_PIN, 1)
        time.sleep(0.5)

    def write(self, command: str) -> None:
        """
        Args:
            command: IM920-HATへ送るcommand文字列。
        Returns:
            なし。
        About:
            BUSY状態の解除を待ち、I2C経由でcommandを書き込む。
        """
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
        """
        Args:
            なし。
        Returns:
            受信済みframe。データがない場合は空文字列。
        About:
            IRQを確認してI2C dataをbufferへ取り込み、最大1件のframeを返す。
        """
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
        """
        Args:
            なし。
        Returns:
            1件のframeをbufferへ格納できた場合はTrue、それ以外はFalse。
        About:
            I2Cから通知長分の文字を読み取り、ring bufferへ保存する。
        """
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
        """
        Args:
            なし。
        Returns:
            なし。
        About:
            I2C busを閉じ、例外の有無にかかわらずGPIO resourceを解放する。
        """
        try:
            self._i2c.close()
        finally:
            GPIO.cleanup()
