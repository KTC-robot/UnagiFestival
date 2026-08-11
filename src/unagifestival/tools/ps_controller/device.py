import logging
import select

from collections.abc import Collection, Sequence
from typing import Final

from evdev import InputDevice, ecodes, list_devices

from unagifestival.tools.ps_controller.models import (
    AxisInfo,
    AxisInfoMap,
    EventCode,
)

logger = logging.getLogger("teensy_log")

type AbsoluteCapabilityEntry = EventCode | tuple[EventCode, AxisInfo]
type SelectResult = tuple[list[int], list[int], list[int]]


# ===== Controller Scoring Constants =====

SCORE_AXIS_PRIMARY: Final[int] = 3
SCORE_AXIS_SECONDARY: Final[int] = 2
SCORE_BTN_PRIMARY: Final[int] = 2
SCORE_BTN_SECONDARY: Final[int] = 1
SCORE_NAME_EXACT: Final[int] = 6
SCORE_NAME_PARTIAL: Final[int] = 3
PENALTY_IGNORED_DEVICE: Final[int] = -100

AXIS_SCORES: Final[dict[EventCode, int]] = {
    ecodes.ABS_X: SCORE_AXIS_PRIMARY,
    ecodes.ABS_Y: SCORE_AXIS_PRIMARY,
    ecodes.ABS_RX: SCORE_AXIS_PRIMARY,
    ecodes.ABS_RY: SCORE_AXIS_PRIMARY,
    ecodes.ABS_HAT0X: SCORE_AXIS_SECONDARY,
    ecodes.ABS_HAT0Y: SCORE_AXIS_SECONDARY,
}
BUTTON_SCORES: Final[dict[EventCode, int]] = {
    ecodes.BTN_SOUTH: SCORE_BTN_PRIMARY,
    ecodes.BTN_EAST: SCORE_BTN_SECONDARY,
    ecodes.BTN_NORTH: SCORE_BTN_SECONDARY,
    ecodes.BTN_WEST: SCORE_BTN_SECONDARY,
}


# ===== Device Name Keywords =====

KEYWORD_WIRELESS: Final[str] = "wireless controller"
KEYWORD_DUALSENSE: Final[str] = "dualsense"
KEYWORD_SONY: Final[str] = "sony"
KEYWORD_TOUCHPAD: Final[str] = "touchpad"
KEYWORD_MOTION: Final[str] = "motion"


def _calculate_device_score(
    device_name: str,
    absolute_codes: Collection[EventCode],
    key_codes: Collection[EventCode],
) -> int:
    """デバイスが持つ機能や名前に基づいてスコアを算出する."""
    score = 0

    if KEYWORD_TOUCHPAD in device_name or KEYWORD_MOTION in device_name:
        score += PENALTY_IGNORED_DEVICE

    score += sum(
        weight for code, weight in AXIS_SCORES.items() if code in absolute_codes
    )
    score += sum(
        weight for code, weight in BUTTON_SCORES.items() if code in key_codes
    )

    if KEYWORD_WIRELESS in device_name:
        score += SCORE_NAME_EXACT
    if KEYWORD_DUALSENSE in device_name:
        score += SCORE_NAME_EXACT
    if KEYWORD_SONY in device_name:
        score += SCORE_NAME_PARTIAL

    return score


def find_controller() -> InputDevice | None:
    """PS5コントローラーを検索する.

    コントローラーが接続されていない場合はエラーにせず None を返す.
    """
    best_device: InputDevice | None = None
    best_score = -1

    for device_path in list_devices():
        try:
            device = InputDevice(device_path)
            capabilities = device.capabilities(absinfo=True)

        except Exception:  # noqa: BLE001
            logger.warning(
                "入力デバイスを読み込めませんでした: %s",
                device_path,
            )
            continue

        absolute_entries: Sequence[AbsoluteCapabilityEntry] = capabilities.get(
            ecodes.EV_ABS,
            [],
        )
        key_entries = capabilities.get(ecodes.EV_KEY, [])

        absolute_codes = {
            entry[0] if isinstance(entry, tuple) else entry
            for entry in absolute_entries
        }

        key_codes = set(key_entries)
        device_name = (device.name or "").lower()

        score = _calculate_device_score(
            device_name,
            absolute_codes,
            key_codes,
        )

        if score > best_score:
            best_score = score
            best_device = device

    if best_device is None:
        logger.warning(
            "PS5コントローラーが接続されていません。コントローラーなしで続行します。"
        )
        return None

    logger.info(
        "Controller: %s %s",
        best_device.path,
        best_device.name,
    )

    return best_device


def get_absolute_axis_info(device: InputDevice) -> AxisInfoMap:
    """デバイスの絶対軸 (ABS) 情報を取得する."""
    capabilities = device.capabilities(absinfo=True)

    axis_information_map: AxisInfoMap = {}

    for entry in capabilities.get(ecodes.EV_ABS, []):
        if isinstance(entry, tuple):
            axis_code, axis_info = entry
            axis_information_map[axis_code] = axis_info

    return axis_information_map


def wait_for_input_ready(
    file_descriptors: Sequence[int],
    timeout_seconds: float = 0.0,
) -> SelectResult:
    """指定されたファイルディスクリプタが読み取り可能になるまで待機する."""
    if not file_descriptors:
        return ([], [], [])

    readable, writable, exceptional = select.select(
        list(file_descriptors),
        [],
        [],
        timeout_seconds,
    )
    return list(readable), list(writable), list(exceptional)
