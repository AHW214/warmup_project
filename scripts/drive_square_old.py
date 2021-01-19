#!/usr/bin/env python3

import math
import rospy
from dataclasses import dataclass
from geometry_msgs.msg import Twist, Vector3
from typing import List


@dataclass
class Movement:
    duration: float
    velocity: Twist


zero_vector: Vector3 = Vector3(0.0, 0.0, 0.0)


def angular_velocity_z(dz: float) -> Twist:
    return Twist(linear=zero_vector, angular=Vector3(0.0, 0.0, dz))


def linear_velocity_x(dx: float) -> Twist:
    return Twist(linear=Vector3(dx, 0, 0), angular=zero_vector)


def send_movements(publisher: rospy.Publisher, moves: List[Movement]) -> None:
    for move in moves:
        publisher.publish(move.velocity)
        rospy.sleep(move.duration)


def drive_square(publisher: rospy.Publisher) -> None:
    drive_line = Movement(duration=10, velocity=linear_velocity_x(0.2))
    turn_corner = Movement(duration=4, velocity=angular_velocity_z(math.pi / 8))

    send_movements(publisher, [drive_line, turn_corner])


def run() -> None:
    rospy.init_node("drive_square")
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    while True:
        drive_square(publisher)


run()
