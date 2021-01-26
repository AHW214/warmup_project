#!/usr/bin/env python3

"""
Have TurtleBot navigate alongside nearby walls.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple, Union
import rospy
from sensor_msgs.msg import LaserScan
from lib.controller import Cmd, Controller, Sub
import lib.mathf as mathf
import lib.turtle_bot as tb

### Model ###


@dataclass
class Approach:
    pass


@dataclass
class Align:
    pass


@dataclass
class Trace:
    init_left_dist: float


Model = Union[Approach, Align, Trace]

init_model: Model = Approach()


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
    left: Optional[tb.ScanPoint]


Msg = Union[Wait, Scan]


def to_msg(scan: LaserScan) -> Msg:
    closest = tb.closest_scan_point(scan)
    forward = tb.scan_point_at(angle_deg=0, scan=scan)
    left = tb.scan_point_at(angle_deg=90, scan=scan)

    return Wait() if closest is None else Scan(closest, forward, left)


### Update ###

VEL_ANGULAR_MIN = 0.0
VEL_ANGULAR_MAX = 2 * math.pi

VEL_LINEAR_MIN = 0.2
VEL_LINEAR_MAX = 2.0

SCAN_RANGE_MAX = 3.5
STOP_DISTANCE = 0.5


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if isinstance(msg, Wait):
        return (model, tb.stop)

    target = msg.closest

    if isinstance(model, Approach):
        if (target := msg.closest).distance < STOP_DISTANCE:
            return (Align(), tb.stop)

        interpolation_angular = abs(target.direction) / math.pi
        direction = mathf.sign(target.direction)
        vel_angular = direction * mathf.lerp(
            low=VEL_ANGULAR_MIN,
            high=VEL_ANGULAR_MAX,
            amount=interpolation_angular,
        )

        interpolation_linear = (1 - 0.45 * interpolation_angular) * min(
            (target.distance - STOP_DISTANCE) / SCAN_RANGE_MAX, 1.0
        )

        vel_linear = mathf.lerp(
            low=VEL_LINEAR_MIN,
            high=VEL_LINEAR_MAX,
            amount=interpolation_linear,
        )

        return (
            model,
            tb.velocity(
                linear=vel_linear,
                angular=vel_angular,
            ),
        )

    if isinstance(model, Align):
        if (
            target.angle_deg > 80 and target.angle_deg < 100 and msg.left is not None
        ):  # TODO
            init_left_dist = msg.left.distance
            return (Trace(init_left_dist), tb.stop)

        return (model, tb.turn_with(vel=-0.25))

    if isinstance(model, Trace):
        to_wall = msg.closest

        # err_left = model.init_left_dist - left.distance
        # interpolation_left = abs(0.0 if abs(err_left) < 0.025 else err_left)
        # direction_left = -1 * mathf.sign(err_left)
        # component_left = direction_left * mathf.lerp(
        #     low=0.0, high=math.pi, amount=interpolation_left
        # )

        diff = 90 - to_wall.angle_deg
        err_angle = -1 * (diff if abs(diff) > 10 else 0.0) / 20

        print(err_angle)

        err_forward = 0.3 * (
            0.0
            if msg.forward is None or msg.forward.distance > 1.0
            else msg.forward.distance
        )
        interpolation_forward = abs(err_forward)
        direction_forward = -1 * mathf.sign(err_forward)
        component_forward = direction_forward * mathf.lerp(
            low=0.0,
            high=2 * math.pi,
            amount=interpolation_forward,
        )

        vel_angular = component_forward if component_forward != 0.0 else err_angle
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
