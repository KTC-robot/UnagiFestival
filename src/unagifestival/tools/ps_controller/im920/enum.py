from enum import IntEnum


class ControlCommand(IntEnum):
    """
    Properties:
        STOP: 通常停止を要求するCommand ID。
        EMERGENCY_STOP: 緊急停止を要求するCommand ID。
        CHANGE_POWER: 走行出力率の変更を要求するCommand ID。
        DRIVE: 車体走行を要求するCommand ID。
        SET_WHEEL_GAIN: 車輪補正gainの設定を要求するCommand ID。
        GAIN_TUNE_START: gain測定走行の開始を要求するCommand ID。
        GAIN_TUNE_KEEPALIVE: gain測定中の通信継続を示すCommand ID。
        GAIN_TUNE_RESULT_ACK: gain測定結果の受信確認を示すCommand ID。
        STEP_ASSIST_RESET: 段差制御状態のresetを要求するCommand ID。

    About:
        Raspberry PiからESP32へ送るCONTROL packet内のCommand IDを定義する。
    """

    STOP = 0x01
    EMERGENCY_STOP = 0x02
    CHANGE_POWER = 0x03
    DRIVE = 0x04
    SET_WHEEL_GAIN = 0x05
    GAIN_TUNE_START = 0x06
    GAIN_TUNE_KEEPALIVE = 0x07
    GAIN_TUNE_RESULT_ACK = 0x08
    STEP_ASSIST_RESET = 0x09


class PacketType(IntEnum):
    """
    Properties:
        CONTROL: 車体制御Commandを格納するpacket種別。
        SERVO_SET: 1つのServo角度を設定するpacket種別。
        SERVO_SET_ALL: 全Servo角度を一括設定するpacket種別。

    About:
        Raspberry PiからESP32へ送信するwire packetの先頭識別値を定義する。
    """

    CONTROL = 0x43
    SERVO_SET = 0x53
    SERVO_SET_ALL = 0x54
