"""モーターPIゲイン調整用の自動走行テストCLI."""

import argparse
import logging
import re
import time

from unagifestival.tools.ps_controller import im_wireless as imw
from unagifestival.tools.ps_controller.config import (
    IM920_CMD_MAX_LEN,
    SLAVE_ADR,
)
from unagifestival.tools.ps_controller.models import DriveCommand
from unagifestival.tools.ps_controller.robot_api import RobotApi
from unagifestival.tools.ps_controller.transport import Im920Transport


type GainSetting = tuple[int, float, float]

DEFAULT_KEEPALIVE_MS = 200
DEFAULT_RESULT_TIMEOUT_MS = 3000

DRIVE_VALUE_MIN = -127
DRIVE_VALUE_MAX = 127

MOTOR_ID_MIN = 1
MOTOR_ID_MAX = 4


def parse_drive_value(value: str) -> int:
    """走行指令値を -127..127 の範囲で検証する."""
    parsed = int(value)

    if not DRIVE_VALUE_MIN <= parsed <= DRIVE_VALUE_MAX:
        message = (
            f"drive value must be "
            f"{DRIVE_VALUE_MIN}..{DRIVE_VALUE_MAX}"
        )
        raise argparse.ArgumentTypeError(message)

    return parsed


def parse_gain(value: str) -> GainSetting:
    """MOTOR:KP:KI 形式のゲイン指定を解析する."""
    parts = value.split(":")

    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "gain must be MOTOR:KP:KI, example: 1:1.0:0.2"
        )

    try:
        motor_id = int(parts[0])
        kp = float(parts[1])
        ki = float(parts[2])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "gain must be MOTOR:KP:KI"
        ) from exc

    if not MOTOR_ID_MIN <= motor_id <= MOTOR_ID_MAX:
        raise argparse.ArgumentTypeError(
            "motor ID must be between 1 and 4"
        )

    if kp < 0.0 or ki < 0.0:
        raise argparse.ArgumentTypeError(
            "Kp and Ki must be >= 0"
        )

    return motor_id, kp, ki


def parse_args() -> argparse.Namespace:
    """CLI引数を解析する."""
    parser = argparse.ArgumentParser(
        description=(
            "Raspberry PiからESP32へゲイン調整用の"
            "自動走行試験を送信する"
        ),
    )

    parser.add_argument(
        "--vx",
        type=parse_drive_value,
        default=20,
        help="前後方向指令 -127..127 (default: 20)",
    )

    parser.add_argument(
        "--vy",
        type=parse_drive_value,
        default=0,
        help="左右方向指令 -127..127 (default: 0)",
    )

    parser.add_argument(
        "--wz",
        type=parse_drive_value,
        default=0,
        help="旋回指令 -127..127 (default: 0)",
    )

    parser.add_argument(
        "--duration-ms",
        type=int,
        default=2000,
        help=(
            "試験時間[ms]。100ms単位、100..10000 "
            "(default: 2000)"
        ),
    )

    parser.add_argument(
        "--gain",
        type=parse_gain,
        action="append",
        default=[],
        metavar="MOTOR:KP:KI",
        help=(
            "試験前に設定するゲイン。複数指定可能。"
            "例: --gain 1:1.2:0.1"
        ),
    )

    parser.add_argument(
        "--keepalive-ms",
        type=int,
        default=DEFAULT_KEEPALIVE_MS,
        help="keepalive送信間隔[ms] (default: 200)",
    )

    parser.add_argument(
        "--result-timeout-ms",
        type=int,
        default=DEFAULT_RESULT_TIMEOUT_MS,
        help=(
            "走行終了後にTUNE DONEを待つ時間[ms] "
            "(default: 3000)"
        ),
    )

    parser.add_argument(
        "--debug",
        action="store_true",
        help="Raspberry Pi側のDEBUGログを有効化",
    )

    return parser.parse_args()


def decode_im920_text(raw: str) -> str:
    """IM920受信フレーム内のhex payloadをUTF-8へ戻す."""
    normalized = "".join(
        chr(ord(character) & 0x7F)
        for character in raw
    ).strip()

    if ":" not in normalized:
        return ""

    payload = normalized.split(":", 1)[1]
    payload = re.sub(r"[^0-9A-Fa-f]", "", payload)

    if len(payload) < 2:
        return ""

    if len(payload) % 2 != 0:
        payload = payload[:-1]

    try:
        return bytes.fromhex(payload).decode(
            "utf-8",
            errors="replace",
        )
    except ValueError:
        return ""


def validate_args(args: argparse.Namespace) -> None:
    """試験条件を検証する."""
    if (
        args.duration_ms < 100
        or args.duration_ms > 10_000
        or args.duration_ms % 100 != 0
    ):
        raise ValueError(
            "--duration-ms must be 100..10000 "
            "and divisible by 100"
        )

    if not 50 <= args.keepalive_ms < 600:
        raise ValueError(
            "--keepalive-ms must be 50..599"
        )

    if args.result_timeout_ms <= 0:
        raise ValueError(
            "--result-timeout-ms must be > 0"
        )


def main() -> None:
    """ゲイン調整用自動走行試験を実行する."""
    args = parse_args()
    validate_args(args)

    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    logger = logging.getLogger("gain_tuning_test")

    im920 = imw.IMWireClass(SLAVE_ADR)

    transport = Im920Transport(
        im920=im920,
        command_max_length=IM920_CMD_MAX_LEN,
        on_transmit=lambda: None,
        logger=logger,
    )

    robot = RobotApi(transport)

    tuning_done = False

    try:
        for motor_id, kp, ki in args.gain:
            print(
                f"ゲイン設定: M{motor_id} "
                f"Kp={kp:.3f} Ki={ki:.3f}"
            )

            robot.set_gain(
                motor_id=motor_id,
                kp=kp,
                ki=ki,
            )

            time.sleep(0.1)

        command = DriveCommand(
            vx=args.vx,
            vy=args.vy,
            wz=args.wz,
        )

        print()
        print("=== ゲイン調整試験開始 ===")
        print(
            f"走行指令: "
            f"vx={command.vx} "
            f"vy={command.vy} "
            f"wz={command.wz}"
        )
        print(f"試験時間: {args.duration_ms} ms")
        print()

        robot.start_gain_tuning(
            command,
            duration_ms=args.duration_ms,
        )

        started = time.monotonic()
        tuning_end = (
            started
            + args.duration_ms / 1000.0
        )

        result_deadline = (
            tuning_end
            + args.result_timeout_ms / 1000.0
        )

        keepalive_interval = (
            args.keepalive_ms / 1000.0
        )

        last_keepalive = started

        while time.monotonic() < result_deadline:
            now = time.monotonic()

            if (
                now < tuning_end
                and now - last_keepalive
                >= keepalive_interval
            ):
                robot.gain_tuning_keepalive()
                last_keepalive = now

            raw = transport.read()

            if raw:
                text = decode_im920_text(raw)

                if text:
                    print(f"ESP32 <- {text}")

                    if text == "TUNE DONE":
                        tuning_done = True
                        break

            time.sleep(0.01)

        if not tuning_done:
            raise TimeoutError(
                "TUNE DONEを受信できませんでした"
            )

        print()
        print("=== ゲイン調整試験完了 ===")

    except KeyboardInterrupt:
        print()
        print("中断しました")

    finally:
        print("STOP送信")

        try:
            robot.stop()
        finally:
            transport.cleanup()


if __name__ == "__main__":
    main()