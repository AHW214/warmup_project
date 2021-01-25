"""
Controller subscriptions.
"""

from dataclasses import dataclass
from typing import Callable, Dict, Generic, List, Type, TypeVar
import rospy

Msg = TypeVar("Msg")


@dataclass
class Sub(Generic[Msg]):
    """
    A subscription to specify inbound ROS messages.
    """

    topic_name: str
    message_type: Type[rospy.Message]
    to_msg: Callable[[rospy.Message], Msg]


class Subscribers(Generic[Msg]):
    """
    A group of ROS subscribers to handle subscriptions.
    """

    sub_dict: Dict[str, rospy.Subscriber] = {}

    def __init__(self, callback: Callable[[Msg], None]) -> None:
        self.callback = callback

    def subscribe(self, subs: List[Sub[Msg]]) -> None:
        """
        Register subscribers for new subscriptions, and unregister subscribers
        for subscriptions no longer present.
        """
        topic_names = list(self.sub_dict.keys())

        for sub in subs:
            if sub.topic_name not in topic_names:
                self.sub_dict[sub.topic_name] = mk_ros_sub(sub, self.callback)

        for topic in topic_names:
            if not any(topic == sub.topic_name for sub in subs):
                self.sub_dict.pop(topic).unregister()


def mk_ros_sub(sub: Sub[Msg], callback: Callable[[Msg], None]) -> rospy.Subscriber:
    """
    Make a ROS subscriber to run a callback for the given subscription.
    """
    return rospy.Subscriber(
        name=sub.topic_name,
        data_class=sub.message_type,
        callback=lambda t: callback(sub.to_msg(t)),
    )
