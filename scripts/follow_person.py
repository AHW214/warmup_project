#!/usr/bin/env python3

"""
Have TurtleBot follow nearby objects.
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
    """
    Possible states: stop or follow.
    """

    stop = 1
    follow = 2


init_model: Model = Model.stop


### Events ###


@dataclass
class Wait:
    """
    Wait while nothing is in range of the scanner.
    """


@dataclass
class Scan:
    """
    Distance and direction to the closest scanned object.
    """

    closest: tb.ScanPoint


Msg = Union[Wait, Scan]


def to_msg(scan: LaserScan) -> Msg:
    """
    Convert LiDAR data to the appropriate message (Wait or Scan).
    """
    closest = tb.closest_scan_point(scan)
    return Wait() if closest is None else Scan(closest)


### Update ###


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if model == Model.follow:
        if isinstance(msg, Wait) or (target := msg.closest).distance < 0.75:
            return (Model.stop, tb.stop)

        (vel_linear, vel_angular) = tb.velocities_to_target(
            target=target,
            angular_dampening=0.45,
            separation_dist=0.75,
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

    # if model == Model.stop:
    if isinstance(msg, Scan) and msg.closest.distance >= 0.75:
        return (Model.follow, None)

    return (model, None)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Msg]]:
    return [tb.laser_scan(to_msg)]


### Run ###


def run() -> None:
    rospy.init_node("follow_person")

    Controller.run(
        model=init_model,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
