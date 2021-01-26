"""
ROS controller commands.
"""

from dataclasses import dataclass
from typing import Dict, Type, TypeVar
import rospy

T = TypeVar("T")


@dataclass
class Cmd:
    """
    A command to specify outbound ROS messages.
    """

    topic_name: str
    message_type: Type[rospy.Message]  # no existential types rip
    message_value: rospy.Message


class Publishers:
    """
    A group of ROS publishers to transmit commands.
    """

    pub_dict: Dict[str, rospy.Publisher] = {}

    def publish(self, cmd: Cmd) -> None:
        """
        Publish the given command and memoize new publishers.
        """
        publisher = self.pub_dict.setdefault(cmd.topic_name, mk_ros_pub(cmd))
        publisher.publish(cmd.message_value)


def mk_ros_pub(cmd: Cmd) -> rospy.Publisher:
    """
    Make a ROS publisher to serve the given command.
    """
    return rospy.Publisher(
        name=cmd.topic_name, data_class=cmd.message_type, queue_size=10
    )
