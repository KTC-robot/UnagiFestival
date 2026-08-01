import serial
import time
import re

COM_PORT = "COM5"   # 自分のIM920-USBのCOM番号に変更
BAUDRATE = 19200

def to_hex(text: str) -> str:
    return text.encode("ascii").hex().upper()

def hex_payload_to_ascii(payload: str) -> str:
    cleaned = re.sub(r"[^0-9A-Fa-f]", "", payload)

    if len(cleaned) < 2:
        return ""

    try:
        return bytes.fromhex(cleaned).decode("ascii", errors="replace")
    except ValueError:
        return ""

def extract_payload_text(line: str) -> str:
    if ":" not in line:
        return ""

    payload = line.rsplit(":", 1)[1]
    return hex_payload_to_ascii(payload)

def send_text(ser: serial.Serial, text: str):
    cmd = f"TXDA {to_hex(text)}\r"
    ser.write(cmd.encode("ascii"))

    print(f"TX TEXT: {text}")
    print(f"CMD -> {cmd.strip()}")

def main():
    print("PC IM920-USB ACK responder")
    print(f"Open {COM_PORT} at {BAUDRATE}bps")

    with serial.Serial(
        COM_PORT,
        BAUDRATE,
        bytesize=8,
        parity=serial.PARITY_NONE,
        stopbits=1,
        timeout=0.2,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    ) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()

        ser.write(b"RDID\r")

        buffer = ""

        while True:
            data = ser.read(1)

            if not data:
                continue

            ch = data.decode("ascii", errors="ignore")

            if ch == "\r" or ch == "\n":
                line = buffer.strip()
                buffer = ""

                if not line:
                    continue

                print(f"IM920 > {line}")

                text = extract_payload_text(line)

                if text:
                    print(f"RX TEXT: {text}")

                    if text.startswith("PING,"):
                        seq = text.split(",", 1)[1].strip()
                        ack = f"ACK,{seq}"
                        send_text(ser, ack)

            else:
                buffer += ch

if __name__ == "__main__":
    main()