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


# Start in the stopped state.
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
    ScanPoint representing the closest scanned object.
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

STOP_DISTANCE = 0.75


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if model == Model.follow:
        # Follow the closest object

        if isinstance(msg, Wait) or (target := msg.closest).distance < STOP_DISTANCE:
            # No scan data or already close to the target

            return (Model.stop, tb.stop)

        # Compute linear and angular velocities for approaching the target

        (vel_linear, vel_angular) = tb.velocities_to_target(
            target=target,
            angular_dampening=0.45,
            separation_dist=STOP_DISTANCE,
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

    # In the stopped state

    if isinstance(msg, Scan) and msg.closest.distance >= STOP_DISTANCE:
        # Follow if there was a scan indicating the object is at a distance

        return (Model.follow, None)

    # For exhaustiveness

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
