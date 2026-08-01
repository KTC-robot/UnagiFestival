# -*- coding: utf-8 -*-

"""
im_wireless.py : IM920-HAT用ライブラリ
GPIO割り込みを使わず、ポーリングで受信確認する版
"""

import time
import smbus
import RPi.GPIO as GPIO


# ピンアサイン BCM
IRQ_PIN = 17       # PIC I2C割り込みピン
XMIT_PIN = 18      # 送信中出力ピン
SLEEP_PIN = 22     # スリープピン
RESET_PIN = 23     # リセットピン
BUSY_PIN = 27      # BUSYピン

RXBUF_MAXSIZE = 0x400


class IMWireClass:
    def __init__(self, sladr):
        self.rxbuf_head = 0
        self.rxbuf_tail = 0
        self.rxbuf_num = 0
        self.rxbuf_maxsize = RXBUF_MAXSIZE
        self.i2c_rxbuf = [0] * self.rxbuf_maxsize

        self.slave_adr = sladr

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(RESET_PIN, GPIO.OUT)
        GPIO.setup(XMIT_PIN, GPIO.IN)
        GPIO.setup(SLEEP_PIN, GPIO.IN)
        GPIO.setup(IRQ_PIN, GPIO.IN)
        GPIO.setup(BUSY_PIN, GPIO.IN)

        GPIO.output(RESET_PIN, 1)

        self.i2c = smbus.SMBus(1)

        # 変換ICのバッファ初期化
        for _ in range(300):
            try:
                self.i2c.read_byte(self.slave_adr)
            except OSError:
                pass

        self.Reboot_920()

    def Reboot_920(self):
        GPIO.output(RESET_PIN, 0)
        time.sleep(0.5)
        GPIO.output(RESET_PIN, 1)
        time.sleep(0.5)

    def Write_920(self, cmd):
        if len(cmd) == 0:
            return

        if cmd[0] != '?':
            while self.busy_status():
                time.sleep(0.001)

        self.i2c.write_i2c_block_data(
            self.slave_adr,
            0,
            [ord(i) for i in cmd]
        )

    def Read_920(self):
        # 割り込みではなく、ここでIRQピンを確認する
        if GPIO.input(IRQ_PIN) == GPIO.HIGH:
            self._read_from_i2c()

        buf = ""

        if self.rxbuf_num >= 1:
            buf = self.i2c_rxbuf[self.rxbuf_head]

            self.rxbuf_head += 1
            self.rxbuf_head &= self.rxbuf_maxsize - 1
            self.rxbuf_num -= 1

        return buf

    def _read_from_i2c(self):
        try:
            i2c_rxlen = self.i2c.read_byte(self.slave_adr)
        except OSError:
            return

        if i2c_rxlen < 1:
            return

        buf = ""

        while i2c_rxlen >= 1:
            try:
                buf += chr(self.i2c.read_byte(self.slave_adr))
            except OSError:
                break

            i2c_rxlen -= 1

        if buf:
            self.i2c_rxbuf[self.rxbuf_tail] = buf

            self.rxbuf_tail += 1
            self.rxbuf_tail &= self.rxbuf_maxsize - 1
            self.rxbuf_num += 1

            if self.rxbuf_num > self.rxbuf_maxsize:
                self.rxbuf_num = self.rxbuf_maxsize

                self.rxbuf_head += 1
                self.rxbuf_head &= self.rxbuf_maxsize - 1

    def irq_intrpt(self, gpio):
        # 互換性用に残しているだけ
        self._read_from_i2c()

    def slp_intrpt(self, gpio):
        pass

    def xmit_intrpt(self, gpio):
        pass

    def remove_interrupt(self, port):
        pass

    def busy_status(self):
        return GPIO.input(BUSY_PIN)

    def gpio_clean(self):
        GPIO.cleanup()
