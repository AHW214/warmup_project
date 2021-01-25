#!/usr/bin/env python3

"""
Have TurtleBot follow nearby objects.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import List, Optional, Tuple
import rospy
from sensor_msgs.msg import LaserScan
from lib.controller import Cmd, Controller, Sub
import lib.mathf as mathf
import lib.turtle_bot as tb


### Model ###
class Model(Enum):
    stop = 1
    follow = 2


init_model: Model = Model.stop


### Events ###


@dataclass
class Scan:
    distance: float
    direction: float


def closest_in_scan(scan: LaserScan) -> Scan:
    (count, distance) = min(enumerate(scan.ranges), key=lambda t: t[1])

    angle = scan.angle_increment * count
    direction = angle - (2 * math.pi if angle > math.pi else 0.0)

    return Scan(distance, direction)


### Update ###


def update(scan: Scan, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if model == Model.follow:
        interpolation = abs(scan.direction) / math.pi
        direction = mathf.sign(scan.direction)
        vel_angular = direction * mathf.smoothstep(0.0, 2 * math.pi, interpolation)

        return (model, tb.velocity(linear=0.0, angular=vel_angular))

    # if isinstance(model, Stop):
    if scan.distance > 0.5:
        return (Model.follow, None)

    return (model, None)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Scan]]:
    return [tb.scan(closest_in_scan)]


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
