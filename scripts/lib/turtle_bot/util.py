"""
TurtleBot3 utilities.
"""

from typing import Callable, Tuple
import lib.mathf as mathf
from lib.turtle_bot.scan import ScanPoint

SCAN_RANGE_MAX = 3.5


def interpolate_signed(
    amount_signed: float,
    output_range: Tuple[float, float],
    interpolator: Callable[[float, float, float], float],
) -> float:
    """
    Interpolate a signed value between two values, retaining the original sign.
    """
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
    """
    Compute the angular velocity to reach `angle_target` from `angle_current`.

    @param `angle_current`: Current angle in degrees.

    @param `angle_target`: Target angle in degrees.

    @param `vel_range`: Minimum and maximum angular velocities to interpolate
    between.

    @param `interpolator`: Function to perform the interpolation (e.g. `lerp` or
    `smoothstep`).
    """
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
    """
    Compute the linear and angular velocities to approach the object at `target`.

    @param `target`: ScanPoint representing the object to approach.

    @param `angular_dampening`: Factor by which to slow linear movement when
    turning, in the range [0, 1].

    @param `separation_dist`: Distance at which to stop approaching the target.

    @param `vel_linear_range`: Minimum and maximum linear velocities to
    interpolate between.

    @param `vel_angular_range`: Minimum and maximum angular velocities to
    interpolate between.

    @param `interpolator`: Function to perform the interpolation (e.g. `lerp` or
    `smoothstep`).

    @return: A tuple of the computed linear and angular velocities (in that order).
    """
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
