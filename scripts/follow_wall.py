#!/usr/bin/env python3

"""
Have TurtleBot navigate alongside nearby walls.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import List, Optional, Tuple, Union
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


def trace_angular_vel(scan: Scan) -> float:
    if scan.forward is None or (wall := scan.forward).distance > 1.0:
        diff_angle = 90 - scan.closest.angle_deg
        sanitized_diff_angle = mathf.zero_abs_under(low=7, value=diff_angle)
        return -0.02 * sanitized_diff_angle

    return -1 * tb.interpolate_signed(
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

        (vel_linear, vel_angular) = tb.velocities_to_target(
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

        vel_angular = tb.vel_angular_to_target(
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
