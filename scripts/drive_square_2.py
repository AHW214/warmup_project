#!/usr/bin/env python3

import lib.vector2 as v2
from dataclasses import dataclass
from lib.turtle_bot import TurtleBot, Transform
from lib.vector2 import Vector2
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


State = Union[Stop, Face, Approach]


class DriveSquare(TurtleBot):
    to_visit: Vector2 = [Vector2(0, 1), Vector2(1, 1), Vector2(1, 0), Vector2(0, 0)]
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

            if abs(d_theta) < 5e-3:
                self.state = Approach(target=self.state.target)
            elif d_theta > 0:
                self.send(velocity_angular=-0.1, velocity_linear=0.0)
            else:
                self.send(velocity_angular=0.1, velocity_linear=0.0)

        elif isinstance(self.state, Approach):
            if approx_equal(transform.position, self.state.target):
                self.send(velocity_angular=0.0, velocity_linear=0.0)
                self.visited.append(self.state.target)
                self.state = Stop()
            else:
                self.send(velocity_angular=0.0, velocity_linear=0.1)


def approx_equal(v: Vector2, w: Vector2) -> bool:
    return v2.sqr_magnitude(v2.sub(v, w)) < 5e-3


def delta_angle(rotation: float, source: Vector2, target: Vector2) -> float:
    direction_target = v2.sub(target, source)
    direction_facing = v2.from_angle(rotation)
    return v2.signed_angle_between(direction_target, direction_facing)


if __name__ == "__main__":
    DriveSquare().run()
