"""ホイールRPM補正ゲイン調整用の自動走行テストCLI."""
# ruff: noqa: C901, EM101, PLR0912, PLR0913, PLR0915, TRY003, T201

import argparse
import csv
import logging
import re
import statistics
import time

from datetime import datetime
from pathlib import Path

from unagifestival.tools.ps_controller import im_wireless as imw
from unagifestival.tools.ps_controller.config import IM920_CMD_MAX_LEN, SLAVE_ADR
from unagifestival.tools.ps_controller.models import DriveCommand
from unagifestival.tools.ps_controller.robot_api import RobotApi
from unagifestival.tools.ps_controller.transport import Im920Transport

type GainSetting = tuple[int, float]
type TuningResult = dict[str, int | str]

DEFAULT_KEEPALIVE_MS = 200
DEFAULT_RESULT_TIMEOUT_MS = 3000
DEFAULT_SPEED = 20
DEFAULT_CSV_PATH = Path("gain_tuning_results.csv")
DRIVE_VALUE_MAX = 127
WHEEL_GAIN_MIN = 0.50
WHEEL_GAIN_MAX = 1.50
MIN_VALID_RPM = 1.0
GAIN_PART_COUNT = 2
HEX_BYTE_CHAR_COUNT = 2
WHEEL_COUNT = 4
TUNING_DURATION_UNIT_MS = 100
TUNING_DURATION_MAX_MS = 10_000
KEEPALIVE_MIN_MS = 50
COMM_TIMEOUT_MS = 600

WHEEL_NAMES = ("FL", "FR", "RL", "RR")
WHEEL_INDEX = {name: index for index, name in enumerate(WHEEL_NAMES)}
WHEEL_MOTOR_ID = (1, 3, 2, 4)
DIRECTION_INDEX = {"forward": 0, "backward": 1, "right": 2, "left": 3}
DIRECTION_VECTORS: dict[str, tuple[int, int, int]] = {
    "forward": (1, 0, 0),
    "backward": (-1, 0, 0),
    "right": (0, 1, 0),
    "left": (0, -1, 0),
}

TUNING_RESULT_PATTERN = re.compile(
    r"^WG(?P<wheel>[0-3]),(?P<mean_rpm>\d+),(?P<sample_count>\d+)"
    r"(?:,(?P<stddev_rpm>\d+))?$"
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
    "wheel",
    "motor_id",
    "current_gain",
    "sample_count",
    "mean_rpm",
    "stddev_rpm",
    "reference_rpm",
    "recommended_gain",
    "received",
    "raw_result",
)


def parse_speed(value: str) -> int:
    parsed = int(value)
    if not 1 <= parsed <= DRIVE_VALUE_MAX:
        raise argparse.ArgumentTypeError("speed must be between 1 and 127")
    return parsed


def parse_gain(value: str) -> GainSetting:
    parts = value.split(":")
    if len(parts) != GAIN_PART_COUNT or parts[0].upper() not in WHEEL_INDEX:
        raise argparse.ArgumentTypeError("gain must be WHEEL:GAIN, example: FL:1.000")
    try:
        gain = float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("gain must be WHEEL:GAIN") from exc
    if not WHEEL_GAIN_MIN <= gain <= WHEEL_GAIN_MAX:
        raise argparse.ArgumentTypeError("gain must be between 0.50 and 1.50")
    return WHEEL_INDEX[parts[0].upper()], gain


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="4輪の実測RPMから方向別wheel gainを算出してCSVへ保存する",
    )
    parser.add_argument("--direction", choices=tuple(DIRECTION_VECTORS), required=True)
    parser.add_argument("--speed", type=parse_speed, default=DEFAULT_SPEED)
    parser.add_argument("--duration-ms", type=int, default=2000)
    parser.add_argument(
        "--gain",
        type=parse_gain,
        action="append",
        default=[],
        metavar="WHEEL:GAIN",
        help="例: --gain FL:1.000 (複数指定可)",
    )
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV_PATH)
    parser.add_argument("--keepalive-ms", type=int, default=DEFAULT_KEEPALIVE_MS)
    parser.add_argument(
        "--result-timeout-ms",
        type=int,
        default=DEFAULT_RESULT_TIMEOUT_MS,
    )
    parser.add_argument("--debug", action="store_true")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if (
        not TUNING_DURATION_UNIT_MS <= args.duration_ms <= TUNING_DURATION_MAX_MS
        or args.duration_ms % TUNING_DURATION_UNIT_MS != 0
    ):
        raise ValueError("--duration-ms must be 100..10000 and divisible by 100")
    if not KEEPALIVE_MIN_MS <= args.keepalive_ms < COMM_TIMEOUT_MS:
        raise ValueError("--keepalive-ms must be 50..599")
    if args.result_timeout_ms <= 0:
        raise ValueError("--result-timeout-ms must be > 0")
    wheel_indexes = [wheel for wheel, _gain in args.gain]
    if len(wheel_indexes) != len(set(wheel_indexes)):
        raise ValueError("--gain may specify each wheel only once")


def build_drive_command(direction: str, speed: int) -> DriveCommand:
    vx_sign, vy_sign, wz_sign = DIRECTION_VECTORS[direction]
    return DriveCommand(vx=vx_sign * speed, vy=vy_sign * speed, wz=wz_sign * speed)


def decode_im920_text(raw: str) -> str:
    """1つのIM920受信フレームを、完全なASCII payloadへ戻す."""
    normalized = "".join(chr(ord(character) & 0x7F) for character in raw).strip()
    if ":" not in normalized:
        return ""
    payload = re.sub(r"[^0-9A-Fa-f]", "", normalized.split(":", 1)[1])
    if len(payload) < HEX_BYTE_CHAR_COUNT or len(payload) % HEX_BYTE_CHAR_COUNT:
        return ""
    try:
        return bytes.fromhex(payload).decode("ascii")
    except (UnicodeDecodeError, ValueError):
        return ""


def parse_tuning_result(text: str) -> TuningResult | None:
    match = TUNING_RESULT_PATTERN.fullmatch(text)
    if match is None:
        return None
    values = match.groupdict()
    return {
        "wheel": int(values["wheel"]),
        "mean_rpm": int(values["mean_rpm"]),
        "sample_count": int(values["sample_count"]),
        "stddev_rpm": int(values["stddev_rpm"]) if values["stddev_rpm"] else 0,
        "raw_result": text,
    }


def gains_by_wheel(gains: list[GainSetting]) -> dict[int, float]:
    result = dict.fromkeys(range(WHEEL_COUNT), 1.0)
    result.update(gains)
    return result


def calculate_recommendations(
    gains: dict[int, float], results: dict[int, TuningResult]
) -> tuple[float | None, dict[int, float]]:
    if set(results) != set(range(WHEEL_COUNT)):
        return None, {}
    rpms = [float(results[wheel]["mean_rpm"]) for wheel in range(WHEEL_COUNT)]
    samples = [int(results[wheel]["sample_count"]) for wheel in range(WHEEL_COUNT)]
    if any(count == 0 for count in samples) or any(rpm <= MIN_VALID_RPM for rpm in rpms):
        return None, {}
    reference_rpm = statistics.mean(rpms)
    recommended = {
        wheel: max(
            WHEEL_GAIN_MIN,
            min(WHEEL_GAIN_MAX, gains[wheel] * reference_rpm / rpms[wheel]),
        )
        for wheel in range(WHEEL_COUNT)
    }
    return reference_rpm, recommended


def write_csv(
    csv_path: Path,
    run_id: str,
    timestamp: str,
    status: str,
    direction: str,
    speed: int,
    command: DriveCommand,
    duration_ms: int,
    gains: dict[int, float],
    results: dict[int, TuningResult],
) -> None:
    reference_rpm, recommendations = calculate_recommendations(gains, results)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not csv_path.exists() or csv_path.stat().st_size == 0
    with csv_path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        if needs_header:
            writer.writeheader()
        for wheel in range(WHEEL_COUNT):
            result = results.get(wheel)
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
                    "wheel": WHEEL_NAMES[wheel],
                    "motor_id": WHEEL_MOTOR_ID[wheel],
                    "current_gain": f"{gains[wheel]:.3f}",
                    "sample_count": result["sample_count"] if result else "",
                    "mean_rpm": result["mean_rpm"] if result else "",
                    "stddev_rpm": result["stddev_rpm"] if result else "",
                    "reference_rpm": f"{reference_rpm:.1f}" if reference_rpm else "",
                    "recommended_gain": (f"{recommendations[wheel]:.3f}" if wheel in recommendations else ""),
                    "received": 1 if result else 0,
                    "raw_result": result["raw_result"] if result else "",
                }
            )


def main() -> None:
    args = parse_args()
    validate_args(args)
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    logger = logging.getLogger("gain_tuning_test")
    command = build_drive_command(args.direction, args.speed)
    gain_map = gains_by_wheel(args.gain)
    started_at = datetime.now().astimezone()
    timestamp = started_at.isoformat(timespec="seconds")
    run_id = started_at.strftime("%Y%m%d_%H%M%S")
    results: dict[int, TuningResult] = {}
    done_received = False
    csv_written = False

    im920 = imw.IMWireClass(SLAVE_ADR)
    transport = Im920Transport(im920, IM920_CMD_MAX_LEN, lambda: None, logger)
    robot = RobotApi(transport)
    try:
        for wheel, gain in args.gain:
            print(f"wheel gain設定: {WHEEL_NAMES[wheel]}={gain:.3f}")
            robot.set_wheel_gain(DIRECTION_INDEX[args.direction], wheel, gain)
            time.sleep(0.1)

        print(f"試験開始: {args.direction} speed={args.speed} duration={args.duration_ms}ms")
        robot.start_gain_tuning(command, args.duration_ms)
        started = time.monotonic()
        tuning_end = started + args.duration_ms / 1000.0
        deadline = tuning_end + args.result_timeout_ms / 1000.0
        keepalive_interval = args.keepalive_ms / 1000.0
        last_keepalive = started

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < tuning_end and now - last_keepalive >= keepalive_interval:
                robot.gain_tuning_keepalive()
                last_keepalive = now
            raw = transport.read()
            if raw:
                logger.debug("[RX RAW] %r", raw)
                text = decode_im920_text(raw)
                if text:
                    logger.debug("[RX DECODED] %s", text)
                    result = parse_tuning_result(text)
                    if result is not None:
                        wheel = int(result["wheel"])
                        results[wheel] = result
                        logger.debug(
                            "[RX RESULT] wheel=%s rpm=%s samples=%s",
                            WHEEL_NAMES[wheel],
                            result["mean_rpm"],
                            result["sample_count"],
                        )
                    elif text == "WD":
                        done_received = True
                if done_received and len(results) == WHEEL_COUNT:
                    break
            time.sleep(0.01)

        missing = set(range(WHEEL_COUNT)) - set(results)
        success = done_received and not missing
        status = "done" if success else "incomplete"
        write_csv(
            args.csv,
            run_id,
            timestamp,
            status,
            args.direction,
            args.speed,
            command,
            args.duration_ms,
            gain_map,
            results,
        )
        csv_written = True
        if not success:
            labels = ", ".join(WHEEL_NAMES[wheel] for wheel in sorted(missing))
            detail = f"missing wheel results: {labels}" if missing else "WD not received"
            raise TimeoutError(detail)

        reference_rpm, recommendations = calculate_recommendations(gain_map, results)
        if reference_rpm is None:
            print("推奨gainは算出不可: sampleCount=0 またはRPMがほぼ0です")
        else:
            print(f"reference RPM: {reference_rpm:.1f}")
            for wheel in range(WHEEL_COUNT):
                print(f"{WHEEL_NAMES[wheel]} recommended gain: {recommendations[wheel]:.3f}")
        print(f"CSV保存: {args.csv}")
    except KeyboardInterrupt:
        if not csv_written:
            write_csv(
                args.csv,
                run_id,
                timestamp,
                "interrupted",
                args.direction,
                args.speed,
                command,
                args.duration_ms,
                gain_map,
                results,
            )
        print("中断しました")
    finally:
        print("STOP送信")
        try:
            robot.stop()
        finally:
            transport.cleanup()


if __name__ == "__main__":
    main()
