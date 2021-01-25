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
    stop = 1
    follow = 2


init_model: Model = Model.stop


### Events ###


@dataclass
class Wait:
    pass


@dataclass
class Scan:
    distance: float
    direction: float


Msg = Union[Wait, Scan]


def to_msg(scan: LaserScan) -> Msg:
    (count, distance) = min(enumerate(scan.ranges), key=lambda t: t[1])

    if math.isinf(distance):
        return Wait()

    angle = scan.angle_increment * count
    direction = angle - (2 * math.pi if angle > math.pi else 0.0)

    return Scan(distance, direction)


### Update ###


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if model == Model.follow:
        if isinstance(msg, Wait) or msg.distance < 0.75:
            return (Model.stop, tb.velocity(linear=0.0, angular=0.0))

        interpolation_angular = abs(msg.direction) / math.pi
        direction = mathf.sign(msg.direction)
        vel_angular = direction * mathf.lerp(
            low=0.0,
            high=2 * math.pi,
            amount=interpolation_angular,
        )

        interpolation_linear = (1 - 0.45 * interpolation_angular) * min(
            (msg.distance - 0.75) / 3.5, 1.0
        )

        vel_linear = mathf.lerp(
            low=0.2,
            high=2.0,
            amount=interpolation_linear,
        )

        return (model, tb.velocity(linear=vel_linear, angular=vel_angular))

    # if isinstance(model, Stop):
    if isinstance(msg, Scan) and msg.distance >= 0.75:
        return (Model.follow, None)

    return (model, None)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Scan]]:
    return [tb.scan(to_msg)]


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
