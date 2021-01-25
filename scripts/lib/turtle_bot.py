"""
TurtleBot3 command and subscription interface.
"""

from dataclasses import dataclass
from typing import Callable
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from lib.controller.controller import Cmd, Msg, Sub
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


def twist_from_velocities(linear_x: float, angular_z: float) -> Twist:
    """
    Create a twist message from the given linear x and angular z velocities.
    """
    return Twist(
        angular=Vector3(x=0.0, y=0.0, z=angular_z),
        linear=Vector3(x=linear_x, y=0.0, z=0.0),
    )


def velocity(linear: float, angular: float) -> Cmd:
    """
    Set the linear and angular velocities of TurtleBot.
    """
    return Cmd(
        topic_name="/cmd_vel",
        message_type=Twist,
        message_value=twist_from_velocities(linear, angular),
    )


def odometry(to_msg: Callable[[Transform], Msg]) -> Sub[Msg]:
    """
    Get odometry information from TurtleBot.
    """
    return Sub(
        topic_name="/odom",
        message_type=Odometry,
        to_msg=lambda odom: to_msg(
            # TODO: compose pattern popping up a bit
            transform_from_odometry(odom)
        ),
    )


def scan(to_msg: Callable[[LaserScan], Msg]) -> Sub[Msg]:
    """
    Get laser scan information from TurtleBot
    """
    return Sub(
        topic_name="/scan",
        message_type=LaserScan,
        to_msg=to_msg,
    )
