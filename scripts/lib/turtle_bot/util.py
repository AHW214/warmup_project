from typing import Callable, Tuple
import lib.mathf as mathf
from lib.turtle_bot.scan import ScanPoint


def interpolate_signed(
    amount_signed: float,
    output_range: Tuple[float, float],
    interpolator: Callable[[float, float, float], float],
) -> float:
    amount = abs(amount_signed)
    sign = mathf.sign(amount_signed)

    return sign * interpolator(*output_range, amount)


def _vel_angular_to_target(
    angle_current: int,
    angle_target: int,
    vel_range: Tuple[float, float],
    interpolator: Callable[[float, float, float], float],
) -> Tuple[float, float]:
    error = (angle_current - angle_target) / 180
    vel_angular = interpolate_signed(error, vel_range, interpolator)

    return (vel_angular, error)


def vel_angular_to_target(
    angle_current: int,
    angle_target: int,
    vel_range: Tuple[float, float],
    interpolator: Callable[[float, float, float], float],
) -> float:
    (vel_angular, _) = _vel_angular_to_target(
        angle_current,
        angle_target,
        vel_range,
        interpolator,
    )
    return vel_angular


def velocities_to_target(
    target: ScanPoint,
    angular_dampening: float,
    separation_dist: float,
    vel_linear_range: Tuple[float, float],
    vel_angular_range: Tuple[float, float],
    interpolator: Callable[[float, float, float], float],
) -> Tuple[float, float]:
    SCAN_RANGE_MAX = 3.5
    (vel_linear_min, vel_linear_max) = vel_linear_range

    (vel_angular, error_angular) = _vel_angular_to_target(
        angle_current=target.angle_deg,
        angle_target=0,
        vel_range=vel_angular_range,
        interpolator=interpolator,
    )

    error_linear = (target.distance - separation_dist) / SCAN_RANGE_MAX
    scale = 1 - angular_dampening * abs(error_angular)
    amount = scale * error_linear

    vel_linear = interpolator(vel_linear_min, vel_linear_max, amount)

    return (vel_linear, vel_angular)
