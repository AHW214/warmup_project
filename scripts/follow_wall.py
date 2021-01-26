#!/usr/bin/env python3

"""
Have TurtleBot navigate alongside nearby walls.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable, List, Optional, Tuple, Union
import rospy
from sensor_msgs.msg import LaserScan
from lib.controller import Cmd, Controller, Sub
import lib.mathf as mathf
import lib.turtle_bot as tb

### Model ###


class Model(Enum):
    approach = 1
    align = 2
    trace = 3


init_model: Model = Model.approach


### Events ###


@dataclass
class Wait:
    """
    Wait while nothing is in range of the scanner.
    """


@dataclass
class Scan:
    closest: tb.ScanPoint
    forward: Optional[tb.ScanPoint]


Msg = Union[Wait, Scan]


def to_msg(scan: LaserScan) -> Msg:
    closest = tb.closest_scan_point(scan)
    forward = tb.scan_point_at(angle_deg=0, scan=scan)

    return Wait() if closest is None else Scan(closest, forward)


### Update ###


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
    target: tb.ScanPoint,
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


def trace_angular_vel(scan: Scan) -> float:
    if scan.forward is None or (wall := scan.forward).distance > 1.0:
        diff_angle = 90 - scan.closest.angle_deg
        sanitized_diff_angle = mathf.zero_abs_under(low=10, value=diff_angle)
        return -0.05 * sanitized_diff_angle

    return -1 * interpolate_signed(
        amount_signed=0.3 * wall.distance,
        output_range=(0.0, 2.0 * math.pi),
        interpolator=mathf.lerp,
    )


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if isinstance(msg, Wait):
        return (model, tb.stop)

    to_wall = msg.closest

    if model == Model.approach:
        if to_wall.distance < 0.5:
            return (Model.align, tb.stop)

        (vel_linear, vel_angular) = velocities_to_target(
            target=to_wall,
            angular_dampening=0.45,
            separation_dist=0.5,
            vel_linear_range=(0.2, 2.0),
            vel_angular_range=(0.0, 2.0 * math.pi),
            interpolator=mathf.lerp,
        )

        return (
            model,
            tb.velocity(
                linear=vel_linear,
                angular=vel_angular,
            ),
        )

    if model == Model.align:
        if to_wall.angle_deg > 88 and to_wall.angle_deg < 92:
            return (Model.trace, tb.stop)

        vel_angular = vel_angular_to_target(
            angle_current=to_wall.angle_deg - 90,
            angle_target=0,
            vel_range=(0.1, math.pi),
            interpolator=mathf.smoothstep,
        )

        return (model, tb.turn_with(vel=vel_angular))

    if model == Model.trace:
        vel_angular = trace_angular_vel(msg)
        return (model, tb.velocity(linear=0.5, angular=vel_angular))

    return (model, None)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Msg]]:
    return [tb.laser_scan(to_msg)]


### Run ###


def run() -> None:
    rospy.init_node("follow_wall")

    Controller.run(
        model=init_model,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
