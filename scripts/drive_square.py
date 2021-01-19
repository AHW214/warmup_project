#!/usr/bin/env python3

import lib.math as math
import lib.vector2 as v2
from dataclasses import dataclass
from lib.turtle_bot import TurtleBot, Transform
from lib.vector2 import Vector2
from math import pi
from typing import List, Union


@dataclass
class Stop:
    pass


@dataclass
class Face:
    target: Vector2


@dataclass
class Approach:
    target: Vector2
    initial_distance: float


State = Union[Stop, Face, Approach]


class DriveSquare(TurtleBot):
    to_visit: List[Vector2] = [
        Vector2(3, 0),
        Vector2(3, 3),
        Vector2(0, 3),
        Vector2(0, 0),
    ]
    visited: List[Vector2] = []
    state: State = Stop()

    def update(self, transform: Transform, deltaTime: float) -> None:
        if isinstance(self.state, Stop):
            if not self.to_visit:
                self.to_visit.extend(self.visited)
                self.visited = []

            target, *rest = self.to_visit
            self.state = Face(target)
            self.to_visit = rest

        elif isinstance(self.state, Face):
            d_theta = delta_angle(
                transform.rotation, transform.position, self.state.target
            )

            if abs(d_theta) < 1e-2:
                target = self.state.target
                distance_to_target = v2.distance_between(transform.position, target)

                self.state = Approach(
                    target=target, initial_distance=distance_to_target
                )
            else:
                sn = -1 * math.sign(d_theta)
                self.send(velocity_angular=sn * 0.5, velocity_linear=0.0)

        elif isinstance(self.state, Approach):
            target = self.state.target
            initial_distance = self.state.initial_distance

            if approx_equal(transform.position, target):
                self.send(velocity_angular=0.0, velocity_linear=0.0)
                self.visited.append(target)
                self.state = Stop()
            else:
                # TODO - this is a mess (and can be refactored)
                half_init_dist = initial_distance / 2
                current_distance = v2.distance_between(transform.position, target)
                midpoint_dist = abs(half_init_dist - current_distance)
                interpolation = 1 - (midpoint_dist / half_init_dist)

                vel_linear = math.smoothstep(0.2, 1, interpolation)

                d_theta = delta_angle(transform.rotation, transform.position, target)

                sn = -1 * math.sign(d_theta)
                angle_thing = abs(d_theta)
                inter = (0 if angle_thing < 1e-2 else angle_thing) / pi
                vel_angular = (
                    sn
                    * max(15 * current_distance, 1)
                    * math.smoothstep(0.0, 2 * pi, inter)
                )

                self.send(velocity_angular=vel_angular, velocity_linear=vel_linear)


def approx_equal(v: Vector2, w: Vector2) -> bool:
    return v2.sqr_magnitude(v2.sub(v, w)) < 1e-3


def delta_angle(rotation: float, source: Vector2, target: Vector2) -> float:
    direction_target = v2.normalize(v2.sub(target, source))
    direction_facing = v2.from_angle(rotation)
    return v2.signed_angle_between(direction_target, direction_facing)


if __name__ == "__main__":
    DriveSquare().run()
