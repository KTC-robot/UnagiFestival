import select

from typing import Final

from evdev import InputDevice, ecodes, list_devices

from unagifestival.tools.ps_controller.handler import logger

# ===== Controller Scoring Constants =====
SCORE_AXIS_PRIMARY: Final[int] = 3
SCORE_AXIS_SECONDARY: Final[int] = 2
SCORE_BTN_PRIMARY: Final[int] = 2
SCORE_BTN_SECONDARY: Final[int] = 1
SCORE_NAME_EXACT: Final[int] = 6
SCORE_NAME_PARTIAL: Final[int] = 3
PENALTY_IGNORED_DEVICE: Final[int] = -100

# ===== Device Name Keywords =====
KEYWORD_WIRELESS: Final[str] = "wireless controller"
KEYWORD_DUALSENSE: Final[str] = "dualsense"
KEYWORD_SONY: Final[str] = "sony"
KEYWORD_TOUCHPAD: Final[str] = "touchpad"
KEYWORD_MOTION: Final[str] = "motion"


def _calculate_device_score(device_name: str, absolute_codes: set, key_codes: set) -> int:  # noqa: C901, PLR0912
    """デバイスが持つ機能 (軸、ボタン) や名前に基づいてスコアを算出する.

    タッチパッドやモーションセンサー等のサブデバイスは除外ペナルティを与える.
    """
    score = 0

    if KEYWORD_TOUCHPAD in device_name or KEYWORD_MOTION in device_name:
        score += PENALTY_IGNORED_DEVICE

    if ecodes.ABS_X in absolute_codes:
        score += SCORE_AXIS_PRIMARY
    if ecodes.ABS_Y in absolute_codes:
        score += SCORE_AXIS_PRIMARY
    if ecodes.ABS_RX in absolute_codes:
        score += SCORE_AXIS_PRIMARY
    if ecodes.ABS_RY in absolute_codes:
        score += SCORE_AXIS_PRIMARY
    if ecodes.ABS_HAT0X in absolute_codes:
        score += SCORE_AXIS_SECONDARY
    if ecodes.ABS_HAT0Y in absolute_codes:
        score += SCORE_AXIS_SECONDARY

    if ecodes.BTN_SOUTH in key_codes:
        score += SCORE_BTN_PRIMARY
    if ecodes.BTN_EAST in key_codes:
        score += SCORE_BTN_SECONDARY
    if ecodes.BTN_NORTH in key_codes:
        score += SCORE_BTN_SECONDARY
    if ecodes.BTN_WEST in key_codes:
        score += SCORE_BTN_SECONDARY

    if KEYWORD_WIRELESS in device_name:
        score += SCORE_NAME_EXACT
    if KEYWORD_DUALSENSE in device_name:
        score += SCORE_NAME_EXACT
    if KEYWORD_SONY in device_name:
        score += SCORE_NAME_PARTIAL

    return score


def find_controller() -> InputDevice:
    """システム上の入力デバイスを走査し、スコアリング結果からメインのゲームコントローラーとして最適なデバイスを取得する.

    Returns:
        InputDevice: 最適なコントローラーデバイス

    Raises:
        RuntimeError: 有効なコントローラーが見つからなかった場合
    """
    best_device = None
    best_score = -1

    for device_path in list_devices():
        try:
            device = InputDevice(device_path)
            capabilities = device.capabilities(absinfo=True)
        except Exception:  # noqa: BLE001
            logger.warning("Some error occurred, but ignore.", exc_info=True)
            continue

        absolute_entries = capabilities.get(ecodes.EV_ABS, [])
        key_entries = capabilities.get(ecodes.EV_KEY, [])

        absolute_codes = {entry[0] if isinstance(entry, tuple) else entry for entry in absolute_entries}
        key_codes = set(key_entries)
        device_name = (device.name or "").lower()

        score = _calculate_device_score(device_name, absolute_codes, key_codes)

        if score > best_score:
            best_score = score
            best_device = device

    if not best_device:
        msg = "No controller device found in /dev/input/event*"
        raise RuntimeError(msg)
    return best_device


def get_absolute_axis_info(device: InputDevice) -> dict:
    """デバイスの絶対軸 (ABS) 情報を取得し、軸コードをキーとした辞書を返す."""
    capabilities = device.capabilities(absinfo=True)
    axis_information_map = {}
    for entry in capabilities.get(ecodes.EV_ABS, []):
        if isinstance(entry, tuple):
            axis_code, axis_info = entry
            axis_information_map[axis_code] = axis_info
    return axis_information_map


def wait_for_input_ready(file_descriptors: list, timeout_seconds: float = 0.0) -> tuple:
    """指定されたファイルディスクリプタが読み取り可能になるまで待機する."""
    return select.select(file_descriptors, [], [], timeout_seconds)
