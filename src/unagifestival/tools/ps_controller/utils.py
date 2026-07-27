from typing import Protocol

from unagifestival.tools.ps_controller.config import (
    AXIS_NORMALIZED_CENTER,
    AXIS_NORMALIZED_MAX,
    AXIS_NORMALIZED_MIN,
)


class AxisInfo(Protocol):
    min: int
    max: int


def clamp_value(value: float, minimum: float, maximum: float) -> float:
    """指定された値を最小値と最大値の範囲内に制限する."""
    return max(minimum, min(maximum, value))


def apply_deadband(value: float, deadband_threshold: float) -> float:
    """値がデッドバンド (不感帯) の閾値未満である場合は0.0を返し、それ以外は元の値を返す."""
    return 0.0 if abs(value) < deadband_threshold else value


def normalize_absolute_axis(value: float, axis_info: AxisInfo | None) -> float:
    """アナログスティックなどの絶対軸の生値を -1.0 から 1.0 の範囲に正規化する.中央値が 0.0 となる."""
    if axis_info is None:
        return AXIS_NORMALIZED_CENTER
    minimum_value, maximum_value = axis_info.min, axis_info.max
    if maximum_value == minimum_value:
        return AXIS_NORMALIZED_CENTER
    center_value = (minimum_value + maximum_value) / 2.0
    half_range = (maximum_value - minimum_value) / 2.0
    return clamp_value(
        (value - center_value) / half_range,
        AXIS_NORMALIZED_MIN,
        AXIS_NORMALIZED_MAX,
    )
