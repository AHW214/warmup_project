"""
A transform to model TurtleBot3.
"""

from dataclasses import dataclass
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from lib.vector2 import Vector2


@dataclass
class Transform:
    """
    Properties specifying the position, rotation, and movement of a TurtleBot.
    """

    position: Vector2
    rotation: float
    velocity_linear: float
    velocity_angular: float


def transform_from_odometry(odom: Odometry) -> Transform:
    """
    Create a TurtleBot transform from the given odometry message.
    """
    pose = odom.pose.pose
    twist = odom.twist.twist

    position = Vector2(
        x=pose.position.x,
        y=pose.position.y,
    )

    (_, _, rotation) = euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    )

    velocity_linear = twist.linear.x
    velocity_angular = twist.angular.z

    return Transform(position, rotation, velocity_linear, velocity_angular)
