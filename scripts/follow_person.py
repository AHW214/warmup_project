#!/usr/bin/env python3

"""
Have TurtleBot follow nearby objects.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple
import rospy
from sensor_msgs.msg import LaserScan
from lib.controller import Cmd, Controller, Sub
import lib.turtle_bot as tb


### Model ###


@dataclass
class Model:
    state: str


init_model: Model = Model(state="")


### Events ###


@dataclass
class Scan:
    distance: float
    direction: float


def closest_in_scan(scan: LaserScan) -> Scan:
    (count, distance) = min(enumerate(scan.ranges), key=lambda t: t[1])
    direction = scan.angle_increment * count
    return Scan(distance, direction)


### Update ###


def update(scan: Scan, model: Model) -> Tuple[Model, Optional[Cmd]]:
    print(scan)
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
