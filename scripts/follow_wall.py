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
class Stop:
    pass


@dataclass
class Approach:
    pass


@dataclass
class Align:
    pass


@dataclass
class Trace:
    pass


Model = Union[Stop, Approach, Align, Trace]

init_model: Model = Stop()


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


Msg = Union[Wait, Scan]


### Update ###


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    pass


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[Msg]]:
    pass


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
