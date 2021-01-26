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
    """
    Possible states: approach the closest wall, align parallel to that wall,
    trace the wall
    """

    approach = 1
    align = 2
    trace = 3


# Start by approaching the closest wall
init_model: Model = Model.approach


### Events ###


@dataclass
class Wait:
    """
    Wait while nothing is in range of the scanner.
    """


@dataclass
class Scan:
    """
    The closest measured ScanPoint, and possibly a ScanPoint of an object
    in the forward direction (an approaching corner)
    """

    closest: tb.ScanPoint
    forward: Optional[tb.ScanPoint]


Msg = Union[Wait, Scan]


def to_msg(scan: LaserScan) -> Msg:
    """
    Convert LiDAR data to the appropriate message.
    """
    closest = tb.closest_scan_point(scan)
    forward = tb.scan_point_at(angle_deg=0, scan=scan)

    return Wait() if closest is None else Scan(closest, forward)


### Update ###

STOP_DISTANCE = 0.5


def trace_angular_vel(scan: Scan) -> float:
    """
    Compute the angular velocity to set when tracing the wall.
    """
    if scan.forward is None or (corner := scan.forward).distance > 1.0:
        # Safely distant from any corners

        # Want angle of closest ScanPoint to be 90 degrees (that of line
        # perpendicular to adjacent wall)

        diff_angle = 90 - scan.closest.angle_deg

        # Filter out values that may be noise

        sanitized_diff_angle = mathf.zero_abs_under(low=7, value=diff_angle)
        return -0.02 * sanitized_diff_angle

    # Approaching a corner; turn more strongly as distance to corner decreases

    return -1 * tb.interpolate_signed(
        amount_signed=1.0 - corner.distance,
        output_range=(0.0, 3.0 * math.pi),
        interpolator=mathf.lerp,
    )


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if isinstance(msg, Wait):
        # Wait for scan data

        return (model, tb.stop)

    to_wall = msg.closest

    if model == Model.approach:
        # Approach the closest wall

        if to_wall.distance < STOP_DISTANCE:
            # Stop and begin aligning with wall

            return (Model.align, tb.stop)

        # Compute linear and angular velocities for approaching the wall

        (vel_linear, vel_angular) = tb.velocities_to_target(
            target=to_wall,
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

    if model == Model.align:
        # Align parallel to the wall

        if to_wall.angle_deg > 88 and to_wall.angle_deg < 92:
            # Approximately parallel; begin tracing the wall

            return (Model.trace, tb.stop)

        # Compute angular velocity for aligning with the wall

        vel_angular = tb.vel_angular_to_target(
            angle_current=to_wall.angle_deg - 90,
            angle_target=0,
            vel_range=(0.1, math.pi),
            interpolator=mathf.smoothstep,
        )

        return (model, tb.turn_with(vel_angular))

    if model == Model.trace:
        # Trace the wall

        # Compute angular velocity for keeping the trace on course (see
        # trace_angular_vel)

        vel_angular = trace_angular_vel(msg)
        return (model, tb.velocity(linear=0.5, angular=vel_angular))

    # For exhaustiveness

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
