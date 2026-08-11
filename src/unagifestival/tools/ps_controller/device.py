import logging
import select

from collections.abc import Collection
from typing import Final

from evdev import AbsInfo, InputDevice, InputEvent, ecodes, list_devices

from unagifestival.tools.ps_controller.enums import (
    AxisCode,
    AxisInputEvent,
    ButtonCode,
    ButtonEvent,
    ButtonState,
    ControllerInputEvent,
    EventType,
)
from unagifestival.tools.ps_controller.models import AxisInfo, AxisInfoMap

logger = logging.getLogger("unagi_log")


# ===== Controller Scoring Constants =====

SCORE_AXIS_PRIMARY: Final[int] = 3
SCORE_AXIS_SECONDARY: Final[int] = 2
SCORE_BTN_PRIMARY: Final[int] = 2
SCORE_BTN_SECONDARY: Final[int] = 1
SCORE_NAME_EXACT: Final[int] = 6
SCORE_NAME_PARTIAL: Final[int] = 3
PENALTY_IGNORED_DEVICE: Final[int] = -100
ABS_CAPABILITY_ENTRY_SIZE: Final[int] = 2

AXIS_SCORES: Final[dict[int, int]] = {
    ecodes.ABS_X: SCORE_AXIS_PRIMARY,
    ecodes.ABS_Y: SCORE_AXIS_PRIMARY,
    ecodes.ABS_RX: SCORE_AXIS_PRIMARY,
    ecodes.ABS_RY: SCORE_AXIS_PRIMARY,
    ecodes.ABS_HAT0X: SCORE_AXIS_SECONDARY,
    ecodes.ABS_HAT0Y: SCORE_AXIS_SECONDARY,
}
BUTTON_SCORES: Final[dict[int, int]] = {
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
    absolute_codes: Collection[int],
    key_codes: Collection[int],
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


def parse_button_state(value: int) -> ButtonState | None:
    """evdevのKEY値をボタン状態へ変換する."""
    if value == ButtonState.RELEASED:
        return ButtonState.RELEASED
    if value == ButtonState.PRESSED:
        return ButtonState.PRESSED

    # evdevのキーリピート値(2)と不正値は操作に変換しない。
    return None


def parse_input_event(event: InputEvent) -> ControllerInputEvent | None:
    """evdevのraw eventを検証してapplication eventへ変換する."""
    if event.type == EventType.ABS:
        axis = AxisCode.get_by_code(event.code)
        if axis is None:
            return None
        return AxisInputEvent(code=axis, value=event.value)

    if event.type == EventType.KEY:
        button = ButtonCode.get_by_code(event.code)
        state = parse_button_state(event.value)
        if button is None or state is None:
            return None
        return ButtonEvent(code=button, state=state)

    return None


def _get_axis_info(device: InputDevice) -> AxisInfoMap:
    """evdevのABS capabilityをapplicationの軸情報へ変換する."""
    axis_information: AxisInfoMap = {}

    for entry in device.capabilities(absinfo=True).get(ecodes.EV_ABS, []):
        if (
            not isinstance(entry, tuple)
            or len(entry) != ABS_CAPABILITY_ENTRY_SIZE
        ):
            continue

        raw_code, raw_info = entry
        if not isinstance(raw_code, int) or not isinstance(raw_info, AbsInfo):
            continue

        axis = AxisCode.get_by_code(raw_code)
        if axis is None:
            continue

        axis_information[axis] = AxisInfo(
            value=raw_info.value,
            minimum=raw_info.min,
            maximum=raw_info.max,
        )

    return axis_information


class ControllerDevice:
    """evdev InputDeviceを型付きcontroller interfaceとして公開する."""

    def __init__(self, device: InputDevice) -> None:
        self._device = device
        self.axis_info = _get_axis_info(device)

    @property
    def path(self) -> str:
        """デバイスパスを返す."""
        return self._device.path

    @property
    def name(self) -> str:
        """デバイス名を返す."""
        return self._device.name or ""

    def grab(self) -> None:
        """コントローラー入力を排他取得する."""
        self._device.grab()

    def ungrab(self) -> None:
        """コントローラー入力の排他取得を解除する."""
        self._device.ungrab()

    def read_events(self, timeout_seconds: float) -> list[ControllerInputEvent]:
        """読み取り可能な入力をapplication eventとして返す."""
        readable, _, _ = select.select(
            [self._device.fd],
            [],
            [],
            timeout_seconds,
        )
        if not readable:
            return []

        events: list[ControllerInputEvent] = []
        for raw_event in self._device.read():
            event = parse_input_event(raw_event)
            if event is not None:
                events.append(event)
        return events


def find_controller() -> ControllerDevice | None:
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
            logger.warning("入力デバイスを読み込めませんでした: %s", device_path)
            continue

        absolute_codes: set[int] = set()
        for entry in capabilities.get(ecodes.EV_ABS, []):
            raw_code = entry[0] if isinstance(entry, tuple) and entry else entry
            if isinstance(raw_code, int):
                absolute_codes.add(raw_code)

        key_codes = {
            raw_code
            for raw_code in capabilities.get(ecodes.EV_KEY, [])
            if isinstance(raw_code, int)
        }
        score = _calculate_device_score(
            (device.name or "").lower(),
            absolute_codes,
            key_codes,
        )

        if score > best_score:
            best_score = score
            best_device = device

    if best_device is None:
        logger.warning("コントローラーが接続されていません。")
        return None

    logger.info("Controller: %s %s", best_device.path, best_device.name)
    return ControllerDevice(best_device)
