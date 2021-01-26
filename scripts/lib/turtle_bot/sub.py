from typing import Callable, TypeVar
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from lib.controller import Sub
from lib.turtle_bot.transform import Transform, transform_from_odometry

Msg = TypeVar("Msg")


def odometry(to_msg: Callable[[Transform], Msg]) -> Sub[Msg]:
    """
    Get odometry information from TurtleBot.
    """
    return Sub(
        topic_name="/odom",
        message_type=Odometry,
        to_msg=lambda odom: to_msg(transform_from_odometry(odom)),
    )


def laser_scan(to_msg: Callable[[LaserScan], Msg]) -> Sub[Msg]:
    """
    Get laser scan information from TurtleBot
    """
    return Sub(
        topic_name="/scan",
        message_type=LaserScan,
        to_msg=to_msg,
    )
