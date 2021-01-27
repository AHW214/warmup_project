#!/usr/bin/env python3

"""
Have TurtleBot follow nearby objects.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import List, Optional, Tuple, Union
from lib.turtle_bot.util import vel_angular_to_target
import rospy
from sensor_msgs.msg import LaserScan
from lib.controller import Cmd, Controller, Sub
import lib.mathf as mathf
import lib.turtle_bot as tb


### Model ###


class Model(Enum):
    """
    Possible states: watch or follow.
    """

    watch = 1
    follow = 2


# Start in the follow state.
init_model: Model = Model.follow


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

WATCH_DISTANCE = 0.75


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if isinstance(msg, Wait):
        # Wait until an object is within range of the scanner

        return (model, tb.stop)

    target = msg.closest

    if model == Model.follow:
        # Follow the closest object

        if target.distance <= WATCH_DISTANCE:
            # Already close to the target

            return (Model.watch, tb.stop)

        # Compute linear and angular velocities for approaching the target

        (vel_linear, vel_angular) = tb.velocities_to_target(
            target=target,
            angular_dampening=0.45,
            separation_dist=WATCH_DISTANCE,
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

    # In the watch state

    if target.distance > WATCH_DISTANCE:
        # Follow if there was a scan indicating the object is at a distance

        return (Model.follow, None)

    # Compute angle to target accounting for scanner noise

    angle_offset = round(mathf.zero_abs_under(10, target.angle_deg))

    vel_angular = vel_angular_to_target(
        angle_current=angle_offset,
        angle_target=0,
        vel_range=(0.0, 2.0 * math.pi),
        interpolator=mathf.lerp,
    )

    # Rotate towards target

    return (model, tb.turn_with(vel_angular))


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
