from dataclasses import dataclass
from typing import Callable, Dict, Generic, List, Type, TypeVar
import rospy

Msg = TypeVar("Msg")


@dataclass
class Sub(Generic[Msg]):
    topic_name: str
    topic_type: Type[rospy.Message]
    topic_to_msg: Callable[[rospy.Message], Msg]


class Subscribers(Generic[Msg]):
    sub_dict: Dict[str, rospy.Subscriber] = {}

    def __init__(self, callback: Callable[[Msg], None]) -> None:
        self.callback = callback

    def update(self, subs: List[Sub[Msg]]) -> None:
        topic_names = list(self.sub_dict.keys())  # TODO: eh

        for s in subs:
            if s.topic_name not in topic_names:
                self.sub_dict[s.topic_name] = mk_ros_sub(s, self.callback)

        for t in topic_names:
            if not any(t == s.topic_name for s in subs):
                self.sub_dict.pop(t).unregister()


def mk_ros_sub(sub: Sub[Msg], callback: Callable[[Msg], None]) -> rospy.Subscriber:
    return rospy.Subscriber(
        name=sub.topic_name,
        data_class=sub.topic_type,
        callback=lambda t: callback(sub.topic_to_msg(t)),
    )
