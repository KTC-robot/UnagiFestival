"""モーターPIゲイン調整用の自動走行テストCLI."""

import argparse
import csv
import logging
import re
import time

from datetime import datetime
from pathlib import Path

from unagifestival.tools.ps_controller import im_wireless as imw
from unagifestival.tools.ps_controller.config import (
    IM920_CMD_MAX_LEN,
    SLAVE_ADR,
)
from unagifestival.tools.ps_controller.models import DriveCommand
from unagifestival.tools.ps_controller.robot_api import RobotApi
from unagifestival.tools.ps_controller.transport import Im920Transport


type GainSetting = tuple[int, float, float]
type TuningResult = dict[str, int | str | None]

DEFAULT_KEEPALIVE_MS = 200
DEFAULT_RESULT_TIMEOUT_MS = 3000
DEFAULT_SPEED = 20
DEFAULT_CSV_PATH = Path("gain_tuning_results.csv")

DRIVE_VALUE_MAX = 127
MOTOR_ID_MIN = 1
MOTOR_ID_MAX = 4

DIRECTION_VECTORS: dict[str, tuple[int, int, int]] = {
    "forward": (1, 0, 0),
    "backward": (-1, 0, 0),
    "right": (0, 1, 0),
    "left": (0, -1, 0),
}

TUNING_RESULT_PATTERN = re.compile(
    r"^TUNE M(?P<motor_id>[1-4])"
    r"(?: MAE=(?P<mae>-?\d+))?"
    r"(?: RMSE=(?P<rmse>-?\d+))?"
    r"(?: MAX=(?P<max_error>-?\d+))?"
    r"(?: FINAL=(?P<final_error>-?\d+))?"
    r"(?: SAT=(?P<sat_percent>\d+))?"
)

CSV_FIELDS = (
    "timestamp",
    "run_id",
    "status",
    "direction",
    "speed",
    "vx",
    "vy",
    "wz",
    "duration_ms",
    "motor_id",
    "kp",
    "ki",
    "mae",
    "rmse",
    "max_error",
    "final_error",
    "sat_percent",
    "received",
    "raw_result",
)


def parse_speed(value: str) -> int:
    """走行速度指令を1..127の範囲で検証する."""
    parsed = int(value)

    if not 1 <= parsed <= DRIVE_VALUE_MAX:
        raise argparse.ArgumentTypeError("speed must be between 1 and 127")

    return parsed


def parse_gain(value: str) -> GainSetting:
    """MOTOR:KP:KI形式のゲイン指定を解析する."""
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
        raise argparse.ArgumentTypeError("Kp and Ki must be >= 0")

    return motor_id, kp, ki


def parse_args() -> argparse.Namespace:
    """CLI引数を解析する."""
    parser = argparse.ArgumentParser(
        description=(
            "Raspberry PiからESP32へゲイン調整用の自動走行試験を送信し、"
            "結果をCSVへ保存する"
        ),
    )

    parser.add_argument(
        "--direction",
        choices=tuple(DIRECTION_VECTORS),
        required=True,
        help="走行方向: forward / backward / right / left",
    )

    parser.add_argument(
        "--speed",
        type=parse_speed,
        default=DEFAULT_SPEED,
        help=f"方向に対する走行指令値 1..127 (default: {DEFAULT_SPEED})",
    )

    parser.add_argument(
        "--duration-ms",
        type=int,
        default=2000,
        help="試験時間[ms]。100ms単位、100..10000 (default: 2000)",
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
        "--csv",
        type=Path,
        default=DEFAULT_CSV_PATH,
        help=f"CSV出力先 (default: {DEFAULT_CSV_PATH})",
    )

    parser.add_argument(
        "--keepalive-ms",
        type=int,
        default=DEFAULT_KEEPALIVE_MS,
        help=f"keepalive送信間隔[ms] (default: {DEFAULT_KEEPALIVE_MS})",
    )

    parser.add_argument(
        "--result-timeout-ms",
        type=int,
        default=DEFAULT_RESULT_TIMEOUT_MS,
        help=(
            "走行終了後にTUNE DONEを待つ時間[ms] "
            f"(default: {DEFAULT_RESULT_TIMEOUT_MS})"
        ),
    )

    parser.add_argument(
        "--debug",
        action="store_true",
        help="Raspberry Pi側のDEBUGログを有効化",
    )

    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    """試験条件を検証する."""
    if (
        args.duration_ms < 100
        or args.duration_ms > 10_000
        or args.duration_ms % 100 != 0
    ):
        raise ValueError(
            "--duration-ms must be 100..10000 and divisible by 100"
        )

    if not 50 <= args.keepalive_ms < 600:
        raise ValueError("--keepalive-ms must be 50..599")

    if args.result_timeout_ms <= 0:
        raise ValueError("--result-timeout-ms must be > 0")


def build_drive_command(direction: str, speed: int) -> DriveCommand:
    """directionとspeedから走行指令を作成する."""
    vx_sign, vy_sign, wz_sign = DIRECTION_VECTORS[direction]

    return DriveCommand(
        vx=vx_sign * speed,
        vy=vy_sign * speed,
        wz=wz_sign * speed,
    )


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
        return bytes.fromhex(payload).decode("utf-8", errors="replace")
    except ValueError:
        return ""


def parse_tuning_result(text: str) -> TuningResult | None:
    """TUNE Mxの結果行をCSV保存用データへ変換する."""
    match = TUNING_RESULT_PATTERN.match(text)

    if match is None:
        return None

    result: TuningResult = {
        "raw_result": text,
    }

    for key, value in match.groupdict().items():
        result[key] = int(value) if value is not None else None

    return result


def gains_by_motor(gains: list[GainSetting]) -> dict[int, tuple[float, float]]:
    """CLI指定ゲインをmotor IDで参照できる辞書へ変換する."""
    return {
        motor_id: (kp, ki)
        for motor_id, kp, ki in gains
    }


def write_csv(
    csv_path: Path,
    run_id: str,
    timestamp: str,
    status: str,
    direction: str,
    speed: int,
    command: DriveCommand,
    duration_ms: int,
    gains: dict[int, tuple[float, float]],
    results: dict[int, TuningResult],
) -> None:
    """1試験分をmotorごとの4行としてCSVへ追記する."""
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not csv_path.exists() or csv_path.stat().st_size == 0

    with csv_path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)

        if needs_header:
            writer.writeheader()

        for motor_id in range(MOTOR_ID_MIN, MOTOR_ID_MAX + 1):
            result = results.get(motor_id)
            gain = gains.get(motor_id)

            writer.writerow(
                {
                    "timestamp": timestamp,
                    "run_id": run_id,
                    "status": status,
                    "direction": direction,
                    "speed": speed,
                    "vx": command.vx,
                    "vy": command.vy,
                    "wz": command.wz,
                    "duration_ms": duration_ms,
                    "motor_id": motor_id,
                    "kp": f"{gain[0]:.3f}" if gain is not None else "",
                    "ki": f"{gain[1]:.3f}" if gain is not None else "",
                    "mae": result.get("mae", "") if result else "",
                    "rmse": result.get("rmse", "") if result else "",
                    "max_error": result.get("max_error", "") if result else "",
                    "final_error": result.get("final_error", "") if result else "",
                    "sat_percent": result.get("sat_percent", "") if result else "",
                    "received": 1 if result is not None else 0,
                    "raw_result": result.get("raw_result", "") if result else "",
                }
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

    command = build_drive_command(args.direction, args.speed)
    gain_map = gains_by_motor(args.gain)

    started_at = datetime.now().astimezone()
    timestamp = started_at.isoformat(timespec="seconds")
    run_id = started_at.strftime("%Y%m%d_%H%M%S")

    results: dict[int, TuningResult] = {}
    tuning_done = False
    tuning_started = False
    csv_written = False

    im920 = imw.IMWireClass(SLAVE_ADR)
    transport = Im920Transport(
        im920=im920,
        command_max_length=IM920_CMD_MAX_LEN,
        on_transmit=lambda: None,
        logger=logger,
    )
    robot = RobotApi(transport)

    try:
        for motor_id, kp, ki in args.gain:
            print(f"ゲイン設定: M{motor_id} Kp={kp:.3f} Ki={ki:.3f}")
            robot.set_gain(motor_id=motor_id, kp=kp, ki=ki)
            time.sleep(0.1)

        print()
        print("=== ゲイン調整試験開始 ===")
        print(f"方向: {args.direction}")
        print(f"速度指令: {args.speed}")
        print(
            f"走行指令: vx={command.vx} "
            f"vy={command.vy} wz={command.wz}"
        )
        print(f"試験時間: {args.duration_ms} ms")
        print()

        robot.start_gain_tuning(command, duration_ms=args.duration_ms)
        tuning_started = True

        started = time.monotonic()
        tuning_end = started + args.duration_ms / 1000.0
        result_deadline = tuning_end + args.result_timeout_ms / 1000.0
        keepalive_interval = args.keepalive_ms / 1000.0
        last_keepalive = started

        while time.monotonic() < result_deadline:
            now = time.monotonic()

            if (
                now < tuning_end
                and now - last_keepalive >= keepalive_interval
            ):
                robot.gain_tuning_keepalive()
                last_keepalive = now

            raw = transport.read()

            if raw:
                text = decode_im920_text(raw)

                if text:
                    print(f"ESP32 <- {text}")

                    result = parse_tuning_result(text)
                    if result is not None:
                        motor_id = int(result["motor_id"])
                        results[motor_id] = result

                    if text == "TUNE DONE":
                        tuning_done = True
                        break

            time.sleep(0.01)

        if not tuning_done:
            write_csv(
                csv_path=args.csv,
                run_id=run_id,
                timestamp=timestamp,
                status="timeout",
                direction=args.direction,
                speed=args.speed,
                command=command,
                duration_ms=args.duration_ms,
                gains=gain_map,
                results=results,
            )
            csv_written = True
            raise TimeoutError("TUNE DONEを受信できませんでした")

        write_csv(
            csv_path=args.csv,
            run_id=run_id,
            timestamp=timestamp,
            status="done",
            direction=args.direction,
            speed=args.speed,
            command=command,
            duration_ms=args.duration_ms,
            gains=gain_map,
            results=results,
        )
        csv_written = True

        print()
        print("=== ゲイン調整試験完了 ===")
        print(f"CSV保存: {args.csv}")

    except KeyboardInterrupt:
        print()
        print("中断しました")

        if tuning_started and not csv_written:
            write_csv(
                csv_path=args.csv,
                run_id=run_id,
                timestamp=timestamp,
                status="interrupted",
                direction=args.direction,
                speed=args.speed,
                command=command,
                duration_ms=args.duration_ms,
                gains=gain_map,
                results=results,
            )
            print(f"途中結果をCSV保存: {args.csv}")

    finally:
        print("STOP送信")

        try:
            robot.stop()
        finally:
            transport.cleanup()


if __name__ == "__main__":
    main()