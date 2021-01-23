from dataclasses import dataclass
from typing import Dict, Optional, Type, TypeVar
import rospy

T = TypeVar("T")


@dataclass
class Cmd:
    topic_name: str
    topic_type: Type[rospy.Message]  # no existential types rip
    topic_value: rospy.Message


class Publishers:
    pub_dict: Dict[str, rospy.Publisher] = {}

    def update(self, cmd: Optional[Cmd]) -> None:
        if cmd is not None:
            publisher = self.pub_dict.setdefault(cmd.topic_name, mk_ros_pub(cmd))
            publisher.publish(cmd.topic_value)


def mk_ros_pub(cmd: Cmd) -> rospy.Publisher:
    return rospy.Publisher(
        name=cmd.topic_name, data_class=cmd.topic_type, queue_size=10
    )
