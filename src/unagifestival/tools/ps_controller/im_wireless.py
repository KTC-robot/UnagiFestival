"""GPIOポーリングでIM920-HATを制御するドライバー."""

import contextlib
import time

from typing import Final

import smbus2 as smbus

from RPi import GPIO

# ピンアサイン BCM
IRQ_PIN: Final[int] = 17  # PIC I2C割り込みピン
XMIT_PIN: Final[int] = 18  # 送信中出力ピン
SLEEP_PIN: Final[int] = 22  # スリープピン
RESET_PIN: Final[int] = 23  # リセットピン
BUSY_PIN: Final[int] = 27  # BUSYピン

RXBUF_MAXSIZE: Final[int] = 0x400


class IMWireClass:
    """IM920-HATをI2C経由で制御するラッパー."""

    def __init__(self, slave_address: int) -> None:
        self.rxbuf_head = 0
        self.rxbuf_tail = 0
        self.rxbuf_num = 0
        self.rxbuf_maxsize = RXBUF_MAXSIZE
        self.i2c_rxbuf: list[str] = [""] * self.rxbuf_maxsize

        self.slave_adr = slave_address

        GPIO.setwarnings(False)  # noqa: FBT003
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(RESET_PIN, GPIO.OUT)
        GPIO.setup(XMIT_PIN, GPIO.IN)
        GPIO.setup(SLEEP_PIN, GPIO.IN)
        GPIO.setup(IRQ_PIN, GPIO.IN)
        GPIO.setup(BUSY_PIN, GPIO.IN)

        GPIO.output(RESET_PIN, 1)

        self.i2c: smbus.SMBus = smbus.SMBus(1)

        # 変換ICのバッファ初期化
        for _ in range(300):
            with contextlib.suppress(OSError):
                self.i2c.read_byte(self.slave_adr)

        self.Reboot_920()

    # 以下の公開メソッド名は既存IM920ドライバーAPIとの互換性を保つ。
    def Reboot_920(self) -> None:  # noqa: N802
        """IM920-HATをハードウェアリセットする."""
        GPIO.output(RESET_PIN, 0)
        time.sleep(0.5)
        GPIO.output(RESET_PIN, 1)
        time.sleep(0.5)

    def Write_920(self, command: str) -> None:  # noqa: N802
        """IM920-HATへ指定したコマンドを送信する."""
        if not command:
            return

        if command[0] != "?":
            while self.busy_status():
                time.sleep(0.001)

        self.i2c.write_i2c_block_data(
            self.slave_adr,
            0,
            [ord(character) for character in command],
        )

    def Read_920(self) -> str:  # noqa: N802
        """IM920-HATから受信済みの1フレームを読み出す."""
        if GPIO.input(IRQ_PIN) == GPIO.HIGH:
            # IRQがHighの間は変換ICに複数フレームが滞留し得る。全件を
            # Python側リングバッファへ移し、呼出しごとに1件ずつ返す。
            for _ in range(64):
                if GPIO.input(IRQ_PIN) != GPIO.HIGH:
                    break
                if not self._read_from_i2c():
                    break

        buffer = ""

        if self.rxbuf_num >= 1:
            buffer = self.i2c_rxbuf[self.rxbuf_head]

            self.rxbuf_head += 1
            self.rxbuf_head &= self.rxbuf_maxsize - 1
            self.rxbuf_num -= 1

        return buffer

    def _read_from_i2c(self) -> bool:
        try:
            received_length = self.i2c.read_byte(self.slave_adr)
        except OSError:
            return False

        if received_length < 1:
            return False

        buffer = ""

        while received_length >= 1:
            try:
                buffer += chr(self.i2c.read_byte(self.slave_adr))
            except OSError:
                # 長さで示されたフレームを読み切れなければpartial frameを
                # 正常データとして公開しない。
                return False

            received_length -= 1

        if buffer:
            self.i2c_rxbuf[self.rxbuf_tail] = buffer

            self.rxbuf_tail += 1
            self.rxbuf_tail &= self.rxbuf_maxsize - 1
            self.rxbuf_num += 1

            if self.rxbuf_num > self.rxbuf_maxsize:
                self.rxbuf_num = self.rxbuf_maxsize

                self.rxbuf_head += 1
                self.rxbuf_head &= self.rxbuf_maxsize - 1

            return True

        return False

    def irq_intrpt(self, _gpio: int) -> None:
        """IRQ割り込みと同じ受信処理を行う互換用callback."""
        self._read_from_i2c()

    def slp_intrpt(self, _gpio: int) -> None:
        """旧スリープ割り込みAPIとの互換用callback."""

    def xmit_intrpt(self, _gpio: int) -> None:
        """旧送信割り込みAPIとの互換用callback."""

    def remove_interrupt(self, _port: int) -> None:
        """旧割り込み解除APIとの互換用メソッド."""

    def busy_status(self) -> int:
        """IM920-HATのBUSYピン状態を返す."""
        return GPIO.input(BUSY_PIN)

    def gpio_clean(self) -> None:
        """GPIOの使用を終了する."""
        GPIO.cleanup()
