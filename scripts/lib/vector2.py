import operator as op
from dataclasses import dataclass
from lib.math import sign
from math import acos, cos, sin, sqrt
from typing import Callable


@dataclass
class Vector2:
    x: float
    y: float


zero: Vector2 = Vector2(x=0.0, y=0.0)

right: Vector2 = Vector2(x=1.0, y=0.0)

up: Vector2 = Vector2(x=0.0, y=1.0)


def _mk_op(
    op: Callable[[float, float], float]
) -> Callable[[Vector2, Vector2], Vector2]:
    return lambda v, w: Vector2(x=op(v.x, w.x), y=op(v.y, w.y))


add: Callable[[Vector2, Vector2], Vector2] = _mk_op(op.add)

sub: Callable[[Vector2, Vector2], Vector2] = _mk_op(op.sub)

mul: Callable[[Vector2, Vector2], Vector2] = _mk_op(op.mul)


def scale(v: Vector2, k: float) -> Vector2:
    return Vector2(x=k * v.x, y=k * v.y)


def dot(v: Vector2, w: Vector2) -> float:
    return v.x * w.x + v.y * w.y


def normalize(v: Vector2) -> Vector2:
    return scale(v, 1 / magnitude(v))


def magnitude(v: Vector2) -> float:
    return sqrt(dot(v, v))


def sqr_magnitude(v: Vector2) -> float:
    return dot(v, v)


def equals(v: Vector2, w: Vector2) -> bool:
    return sqr_magnitude(sub(v, w)) < 1e-6


def angle_between(v: Vector2, w: Vector2) -> float:
    return acos(dot(v, w) / (magnitude(v) * magnitude(w)))


def distance_between(v: Vector2, w: Vector2) -> float:
    return magnitude(sub(v, w))


def signed_angle_between(v: Vector2, w: Vector2) -> float:
    unsigned_angle = angle_between(v, w)
    sn = sign(v.x * w.y - v.y * w.x)
    return sn * unsigned_angle


def from_angle(angle: float) -> Vector2:
    return Vector2(cos(angle), sin(angle))
