"""ホイールRPM補正ゲイン調整用の自動走行テストCLI."""
# CLIの結果は標準出力へ表示するため、print禁止だけをファイル単位で除外する。
# ruff: noqa: T201

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

DEFAULT_KEEPALIVE_MS = 100
# result ACKのbounded retryをPi側が待ち切れるよう、走行終了後に余裕を持たせる。
DEFAULT_RESULT_TIMEOUT_MS = 6000
DEFAULT_GAIN_ACK_TIMEOUT_MS = 3000
DEFAULT_SPEED = 20
DEFAULT_CSV_PATH = Path("gain_tuning_results.csv")
DRIVE_VALUE_MAX = 127
WHEEL_GAIN_MIN = 0.50
WHEEL_GAIN_MAX = 1.50
MIN_VALID_RPM = 1.0
GAIN_PART_COUNT = 2
GAIN_ACK_ATTEMPT_TIMEOUT_SEC = 0.5
GAIN_ACK_RETRY_COUNT = 3
GAIN_TX_TURNAROUND_GUARD_SEC = 0.15
RESULT_ACK_SETTLE_SEC = 0.7
HEX_BYTE_CHAR_COUNT = 2
WHEEL_COUNT = 4
TUNING_DURATION_UNIT_MS = 100
TUNING_DURATION_MAX_MS = 10_000
KEEPALIVE_MIN_MS = 50
COMM_TIMEOUT_MS = 600
MIN_RECOMMENDED_SAMPLE_COUNT = 10

INVALID_SPEED_MESSAGE = "speed must be between 1 and 127"
INVALID_GAIN_FORMAT_MESSAGE = "gain must be WHEEL:GAIN, example: FL:1.000"
INVALID_GAIN_RANGE_MESSAGE = "gain must be between 0.50 and 1.50"
INVALID_DURATION_MESSAGE = "--duration-ms must be 100..10000 and divisible by 100"
INVALID_KEEPALIVE_MESSAGE = "--keepalive-ms must be 50..599"
INVALID_RESULT_TIMEOUT_MESSAGE = "--result-timeout-ms must be > 0"
DUPLICATE_GAIN_MESSAGE = "--gain may specify each wheel only once"

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
WHEEL_GAIN_ACK_PATTERN = re.compile(r"^WGS,(?P<direction>[0-3]),(?P<wheel>[0-3]),(?P<gain>\d+(?:\.\d+)?)$")

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
    """速度引数を1-127の整数として解析する.

    Raises:
        argparse.ArgumentTypeError: 指定値が許容範囲外の場合.
    """
    parsed = int(value)
    if not 1 <= parsed <= DRIVE_VALUE_MAX:
        raise argparse.ArgumentTypeError(INVALID_SPEED_MESSAGE)
    return parsed


def parse_gain(value: str) -> GainSetting:
    """`WHEEL:GAIN`形式をwheel indexとgainへ変換する.

    Args:
        value: 例として`FL:1.000`形式の文字列.

    Returns:
        wheel indexと0.50-1.50のgain.

    Raises:
        argparse.ArgumentTypeError: 形式、wheel名、gain範囲が不正な場合.
    """
    parts = value.split(":")
    if len(parts) != GAIN_PART_COUNT or parts[0].upper() not in WHEEL_INDEX:
        raise argparse.ArgumentTypeError(INVALID_GAIN_FORMAT_MESSAGE)
    try:
        gain = float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(INVALID_GAIN_FORMAT_MESSAGE) from exc
    if not WHEEL_GAIN_MIN <= gain <= WHEEL_GAIN_MAX:
        raise argparse.ArgumentTypeError(INVALID_GAIN_RANGE_MESSAGE)
    return WHEEL_INDEX[parts[0].upper()], gain


def parse_args() -> argparse.Namespace:
    """コマンドライン引数を定義して解析する."""
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
    """試験時間、keepalive、timeout、gain指定の整合性を検証する.

    Args:
        args: `parse_args`が返したCLI引数.

    Raises:
        ValueError: 制約外の値またはwheelの重複指定がある場合.
    """
    if (
        not TUNING_DURATION_UNIT_MS <= args.duration_ms <= TUNING_DURATION_MAX_MS
        or args.duration_ms % TUNING_DURATION_UNIT_MS != 0
    ):
        raise ValueError(INVALID_DURATION_MESSAGE)
    if not KEEPALIVE_MIN_MS <= args.keepalive_ms < COMM_TIMEOUT_MS:
        raise ValueError(INVALID_KEEPALIVE_MESSAGE)
    if args.result_timeout_ms <= 0:
        raise ValueError(INVALID_RESULT_TIMEOUT_MESSAGE)
    wheel_indexes = [wheel for wheel, _gain in args.gain]
    if len(wheel_indexes) != len(set(wheel_indexes)):
        raise ValueError(DUPLICATE_GAIN_MESSAGE)


def build_drive_command(direction: str, speed: int) -> DriveCommand:
    """ユーザー指定の方向と速度からESP32向け走行指令を作成する.

    Args:
        direction: forward、backward、right、leftのいずれか.
        speed: 方向へ掛ける1-127の速度値.

    Returns:
        符号を反映したvx、vy、wzを持つ走行指令.
    """
    vx_sign, vy_sign, wz_sign = DIRECTION_VECTORS[direction]
    return DriveCommand(vx=vx_sign * speed, vy=vy_sign * speed, wz=wz_sign * speed)


def decode_im920_text(raw: str) -> str:
    """1つのIM920受信フレームを完全なASCII payloadへ戻す.

    奇数桁hexやASCIIでないpayloadはpartial frameとして空文字を返す.

    Args:
        raw: node情報とhex payloadを含むIM920受信文字列.

    Returns:
        復号したpayload。完全に復号できない場合は空文字.
    """
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
    """`WG<wheel>,rpm,samples[,stddev]`を試験結果へ変換する.

    Args:
        text: ESP32から受信した復号済みpayload.

    Returns:
        wheel RPM結果。WG形式と完全一致しない場合はNone.
    """
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
    """CLI指定を4輪すべてのcurrent gainへ展開する.

    未指定wheelは1.0とする。この戻り値は試験前に全件ESP32へ送信し、
    Python側の認識値とESP32実値を毎試験一致させる.

    Args:
        gains: CLIで明示されたwheel indexとgain.

    Returns:
        0-3の全wheelを必ず含むgain辞書.
    """
    result = dict.fromkeys(range(WHEEL_COUNT), 1.0)
    result.update(gains)
    return result


def calculate_recommendations(
    gains: dict[int, float], results: dict[int, TuningResult]
) -> tuple[float | None, dict[int, float]]:
    """4輪の平均絶対RPMから次回の推奨gainを計算する.

    Args:
        gains: 今回ESP32へ適用確認済みのcurrent gain.
        results: wheel index別のRPM集計結果.

    Returns:
        全結果が有効ならreference RPMと0.50-1.50へclampした推奨gain.
        欠損、sampleCount=0、またはRPMがほぼ0なら`(None, {})`.
    """
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


def read_decoded_frame(
    transport: Im920Transport,
    logger: logging.Logger,
) -> str:
    """IM920から最大1フレームを読み、debugログ付きで復号する."""
    raw = transport.read()
    if not raw:
        return ""
    logger.debug("[RX RAW] %r", raw)
    text = decode_im920_text(raw)
    if text:
        logger.debug("[RX DECODED] %s", text)
    return text


def set_and_confirm_wheel_gains(
    robot: RobotApi,
    transport: Im920Transport,
    logger: logging.Logger,
    direction: int,
    gains: dict[int, float],
    timeout_ms: int = DEFAULT_GAIN_ACK_TIMEOUT_MS,
) -> None:
    """4輪のgainを送信し、ESP32が返すWGSを全件照合する.

    TXDAのローカルOKでは遠端ESP32への適用を確認できないため、direction、
    wheel、wire scale丸め後のgainが一致するWGSだけをACKとして扱う.

    Args:
        robot: wheel gain設定API.
        transport: WGSを受信するIM920 transport.
        logger: debugログ出力先.
        direction: ESP32 wire protocol上の方向番号.
        gains: 0-3の4wheelすべてを含むgain.
        timeout_ms: 全WGSを待つ最大時間[ms].

    Raises:
        TimeoutError: timeoutまでに正しいWGSが4輪分揃わない場合.
    """
    overall_deadline = time.monotonic() + timeout_ms / 1000.0
    # 未指定wheelを含む4輪すべてを毎試験送信し、ESP32実値とcurrent_gainを同期する。
    for wheel in range(WHEEL_COUNT):
        gain = gains[wheel]

        acknowledged = False

        for attempt in range(GAIN_ACK_RETRY_COUNT):
            if time.monotonic() >= overall_deadline:
                break
            logger.debug(
                "[GAIN TX] wheel=%s attempt=%d",
                WHEEL_NAMES[wheel],
                attempt + 1,
            )

            robot.set_wheel_gain(
                direction,
                wheel,
                gain,
            )

            deadline = min(
                overall_deadline,
                time.monotonic() + GAIN_ACK_ATTEMPT_TIMEOUT_SEC,
            )

            while time.monotonic() < deadline:
                text = read_decoded_frame(
                    transport,
                    logger,
                )

                match = WHEEL_GAIN_ACK_PATTERN.fullmatch(text)

                if match is None:
                    time.sleep(0.01)
                    continue

                ack_direction = int(match.group("direction"))
                ack_wheel = int(match.group("wheel"))
                ack_scaled = round(float(match.group("gain")) * 1000)

                if ack_direction == direction and ack_wheel == wheel and ack_scaled == round(gain * 1000):
                    acknowledged = True
                    break

            if acknowledged:
                # ESP32 -> Pi送信直後に逆方向へ送らず、
                # 無線の送受信切替時間を確保する。
                time.sleep(GAIN_TX_TURNAROUND_GUARD_SEC)
                break

            logger.warning(
                "[GAIN ACK] timeout wheel=%s attempt=%d",
                WHEEL_NAMES[wheel],
                attempt + 1,
            )

        if not acknowledged:
            detail = f"wheel gain acknowledgement timeout; missing: {WHEEL_NAMES[wheel]}"
            raise TimeoutError(detail)


def collect_tuning_results(
    robot: RobotApi,
    transport: Im920Transport,
    logger: logging.Logger,
    duration_ms: int,
    result_timeout_ms: int,
    keepalive_ms: int,
) -> tuple[dict[int, TuningResult], bool]:
    """走行中のkeepaliveを維持しながらWG0-WG3とWDを回収する.

    Args:
        robot: keepalive送信用API.
        transport: 結果受信用transport.
        logger: debugログ出力先.
        duration_ms: ESP32側の試験時間[ms].
        result_timeout_ms: 試験終了後の結果待ち時間[ms].
        keepalive_ms: 走行中のkeepalive間隔[ms].

    Returns:
        wheel別結果とWD受信有無. WG全件とWDが揃うまで待機する.
    """
    results: dict[int, TuningResult] = {}
    done_received = False
    completion_deadline: float | None = None
    started = time.monotonic()
    tuning_end = started + duration_ms / 1000.0
    deadline = tuning_end + result_timeout_ms / 1000.0
    keepalive_interval = keepalive_ms / 1000.0
    last_keepalive = started

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now < tuning_end and now - last_keepalive >= keepalive_interval:
            robot.gain_tuning_keepalive()
            last_keepalive = now

        text = read_decoded_frame(transport, logger)
        result = parse_tuning_result(text)
        if result is not None:
            wheel = int(result["wheel"])
            # ACK欠損によるduplicate WGも上書き保存し、必ず再ACKする。
            results[wheel] = result
            logger.debug(
                "[RX RESULT] wheel=%s rpm=%s samples=%s",
                WHEEL_NAMES[wheel],
                result["mean_rpm"],
                result["sample_count"],
            )
            robot.ack_gain_tuning_result(wheel)
            logger.debug("[RESULT ACK TX] WG%s", wheel)
        elif text == "WD":
            done_received = True
            logger.debug("[RX DONE] WD")
            robot.ack_gain_tuning_result(WHEEL_COUNT)
            logger.debug("[RESULT ACK TX] WD")

        # WDだけでは成功にせず、wheel番号をsequenceとして完全性を確認する。
        if done_received and len(results) == WHEEL_COUNT and completion_deadline is None:
            # ACK4欠損時に再送されるWDにも応答できる短い受信猶予を設ける。
            completion_deadline = time.monotonic() + RESULT_ACK_SETTLE_SEC
            deadline = max(deadline, completion_deadline)
        if completion_deadline is not None and time.monotonic() >= completion_deadline:
            break
        time.sleep(0.01)

    return results, done_received


def tuning_completion_error(
    results: dict[int, TuningResult],
    *,
    done_received: bool,
) -> str | None:
    """結果protocolと有効sampleの不足理由を返す.

    WG0-WG3、WD、有効sampleのすべてが揃う場合だけNoneを返す.
    """
    missing = set(range(WHEEL_COUNT)) - set(results)
    if missing:
        labels = ", ".join(WHEEL_NAMES[wheel] for wheel in sorted(missing))
        return f"missing wheel results: {labels}"
    if not done_received:
        return "WD not received"

    zero_sample_wheels = {wheel for wheel, result in results.items() if int(result["sample_count"]) == 0}
    if zero_sample_wheels:
        labels = ", ".join(WHEEL_NAMES[wheel] for wheel in sorted(zero_sample_wheels))
        return f"no valid RPM samples: {labels}"
    return None


def write_csv(  # noqa: PLR0913
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
    """1回の試験結果をwheelごとの4行としてCSVへ追記する.

    Args:
        csv_path: 出力先CSV.
        run_id: 試験を識別するID.
        timestamp: 試験開始時刻.
        status: done、incomplete、interruptedのいずれか.
        direction: ユーザー指定の走行方向.
        speed: 走行速度指令.
        command: 実際に送ったvx、vy、wz.
        duration_ms: 試験時間[ms].
        gains: ESP32への適用をWGSで確認したcurrent gain.
        results: 受信できたwheel別RPM結果.
    """
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


def print_recommendations(
    args: argparse.Namespace,
    gains: dict[int, float],
    results: dict[int, TuningResult],
) -> None:
    """測定結果、推奨gain、次回用CLIコマンドを表示する."""
    reference_rpm, recommendations = calculate_recommendations(gains, results)
    print(f"direction: {args.direction}")
    if reference_rpm is None:
        print("推奨gainは算出不可: sampleCount=0 またはRPMがほぼ0です")
        return

    print(f"reference RPM: {reference_rpm:.1f}")
    print()
    for wheel in range(WHEEL_COUNT):
        sample_count = int(results[wheel]["sample_count"])
        print(
            f"{WHEEL_NAMES[wheel]} current={gains[wheel]:.3f} "
            f"rpm={int(results[wheel]['mean_rpm'])} "
            f"recommended={recommendations[wheel]:.3f}"
        )
        # 5ms制御周期で10sample未満は測定区間が50ms未満の目安になるため警告のみ行う。
        if sample_count < MIN_RECOMMENDED_SAMPLE_COUNT:
            print(f"WARNING: {WHEEL_NAMES[wheel]}の有効sampleが少ないです ({sample_count})")

    gain_options = " ".join(f"--gain {WHEEL_NAMES[wheel]}:{recommendations[wheel]:.3f}" for wheel in range(WHEEL_COUNT))
    print()
    print(f"次回推奨:\n{gain_options}")
    print()
    print("次回実行コマンド:")
    print("uv run gain-tuning-test \\")
    print(f"  --direction {args.direction} \\")
    print(f"  --speed {args.speed} \\")
    print(f"  --duration-ms {args.duration_ms} \\")
    for wheel in range(WHEEL_COUNT):
        suffix = " \\" if wheel < WHEEL_COUNT - 1 else ""
        print(f"  --gain {WHEEL_NAMES[wheel]}:{recommendations[wheel]:.3f}{suffix}")


def main() -> None:
    """4輪gain同期、走行試験、結果検証、CSV保存を順に実行する.

    WGSを4輪分確認できなければ走行を開始しない。走行開始後は例外や
    ユーザー中断を含むすべての終了経路でSTOPとGPIO cleanupを実行する.
    """
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
    csv_written = False

    im920 = imw.IMWireClass(SLAVE_ADR)
    transport = Im920Transport(im920, IM920_CMD_MAX_LEN, lambda: None, logger)
    robot = RobotApi(transport)
    try:
        set_and_confirm_wheel_gains(
            robot,
            transport,
            logger,
            DIRECTION_INDEX[args.direction],
            gain_map,
        )

        print(f"試験開始: {args.direction} speed={args.speed} duration={args.duration_ms}ms")
        logger.debug("[TUNE START] command=%s duration_ms=%s", command, args.duration_ms)
        robot.start_gain_tuning(command, args.duration_ms)
        results, done_received = collect_tuning_results(
            robot,
            transport,
            logger,
            args.duration_ms,
            args.result_timeout_ms,
            args.keepalive_ms,
        )

        completion_error = tuning_completion_error(
            results,
            done_received=done_received,
        )
        success = completion_error is None
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
        if completion_error is not None:
            raise TimeoutError(completion_error)

        print_recommendations(args, gain_map, results)
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
