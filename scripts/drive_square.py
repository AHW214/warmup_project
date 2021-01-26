#!/usr/bin/env python3

"""
Have TurtleBot drive in a square.
"""

from dataclasses import dataclass, replace
import math
from typing import List, Optional, Tuple, Union
import rospy
from lib.controller import Cmd, Controller, Sub
import lib.mathf as mathf
import lib.turtle_bot as tb
from lib.vector2 import Vector2
import lib.vector2 as v2

### Model ###


@dataclass
class Stop:
    """
    Stop moving.
    """

    until_time: rospy.Time


@dataclass
class Face:
    """
    Face the target.
    """

    target: Vector2


@dataclass
class Approach:
    """
    Approach the target.
    """

    target: Vector2
    target_midpoint: Vector2
    midpoint_dist: float


State = Union[Stop, Face, Approach]


@dataclass
class Model:
    """
    Model for tracing a polygonal path.
    """

    to_visit: List[Vector2]
    visited: List[Vector2]
    state: State


init_model: Model = Model(
    to_visit=[
        Vector2(3, 0),
        Vector2(3, 3),
        Vector2(0, 3),
        Vector2(0, 0),
    ],
    visited=[],
    state=Stop(until_time=rospy.Time.from_sec(0.0)),
)


def delta_angle(transform: tb.Transform, target: Vector2) -> float:
    """
    Compute the angle between the direction of the TurtleBot and the direction
    to the target position.
    """
    direction_target = v2.normalize(target - transform.position)
    direction_facing = v2.from_angle(transform.rotation)
    return v2.signed_angle_between(direction_target, direction_facing)


### Update ###


def update(transform: tb.Transform, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if isinstance(model.state, Face):
        # Face the target position

        target = model.state.target
        d_theta = delta_angle(transform, target)

        if abs(d_theta) < 1e-2:
            # Approximately facing the target

            target_midpoint = v2.scale(transform.position + target, 0.5)
            midpoint_dist = v2.distance_between(transform.position, target_midpoint)

            # Begin approaching target

            new_model = replace(
                model,
                state=Approach(
                    target=target,
                    target_midpoint=target_midpoint,
                    midpoint_dist=midpoint_dist,
                ),
            )

            return (new_model, None)

        # Rotate to face target if not yet aligned

        direction = -1 * mathf.sign(d_theta)
        vel_angular = direction * 0.5

        return (model, tb.turn_with(vel_angular))

    if isinstance(model.state, Approach):
        # Approach the target position

        target = model.state.target
        midpoint = model.state.target_midpoint
        midpoint_dist = model.state.midpoint_dist

        target_dist = v2.distance_between(transform.position, target)

        if target_dist < 3e-2:
            # Approximately at the target; pause movement

            new_model = replace(
                model,
                visited=[*model.visited, target],
                state=Stop(until_time=rospy.Time.now() + rospy.Duration.from_sec(0.5)),
            )

            return (new_model, tb.stop)

        # Compute linear velocity to target based on current distance

        new_dist = v2.distance_between(transform.position, midpoint)
        interpolation_linear = 1.0 - (new_dist / midpoint_dist)
        vel_linear = mathf.smoothstep(0.2, 1.0, interpolation_linear)

        # Computer angular velocity to target based on current rotation
        # and distance

        d_theta = delta_angle(transform, target)
        scale = max(15 * target_dist, 1.0)

        vel_angular = (
            -1
            * scale
            * tb.vel_angular_to_target(
                angle_current=math.degrees(d_theta),
                angle_target=0,
                vel_range=(0.0, 2.0 * math.pi),
                interpolator=mathf.smoothstep,
            )
        )

        return (model, tb.velocity(linear=vel_linear, angular=vel_angular))

    # In the stop state; wait until stop time has passed

    if rospy.Time.now() < model.state.until_time:
        return (model, None)

    # Dequeue next target, or reset targets if all have been visited

    ((target, *to_visit), visited) = (
        (model.to_visit, model.visited) if model.to_visit else (model.visited, [])
    )

    new_model = Model(
        to_visit=to_visit,
        visited=visited,
        state=Face(target),
    )

    # For exhaustiveness

    return (new_model, None)


### Subscriptions ###


def subscriptions(_: Model) -> List[Sub[tb.Transform]]:
    return [tb.odometry(lambda t: t)]


### Run ###


def run() -> None:
    rospy.init_node("drive_square")

    Controller.run(
        model=init_model,
        update=update,
        subscriptions=subscriptions,
    )

    rospy.spin()


if __name__ == "__main__":
    run()
