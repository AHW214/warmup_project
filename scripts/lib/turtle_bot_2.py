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
    position: Vector2
    rotation: float
    velocity_linear: float
    velocity_angular: float


def transform_from_odometry(odom: Odometry) -> Transform:
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


def twist_from_velocities(linear: float, angular: float) -> Twist:
    return Twist(
        angular=Vector3(x=0.0, y=0.0, z=angular),
        linear=Vector3(x=linear, y=0.0, z=0.0),
    )


def velocity(linear: float, angular: float) -> Cmd:
    return Cmd(
        topic_name="/cmd_vel",
        topic_type=Twist,
        topic_value=twist_from_velocities(linear, angular),
    )


def odometry(to_msg: Callable[[Transform], Msg]) -> Sub[Msg]:
    return Sub(
        topic_name="/odom",
        topic_type=Odometry,
        topic_to_msg=lambda odom: to_msg(
            # TODO: compose pattern popping up a bit
            transform_from_odometry(odom)
        ),
    )


def scan(to_msg: Callable[[LaserScan], Msg]) -> Sub[Msg]:
    return Sub(
        topic_name="/scan",
        topic_type=LaserScan,
        topic_to_msg=to_msg,
    )
