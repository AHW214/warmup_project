"""
ROS controller.
"""

from queue import Queue
from threading import Thread
from typing import Callable, Generic, List, Optional, Tuple, TypeVar
from lib.controller.cmd import Cmd, Publishers
from lib.controller.sub import Sub, Subscribers


Model = TypeVar("Model")
Msg = TypeVar("Msg")

Update = Callable[[Msg, Model], Tuple[Model, Optional[Cmd]]]
Subscriptions = Callable[[Model], List[Sub[Msg]]]


class Controller(Generic[Model, Msg]):
    """
    A controller for communicating with ROS nodes.
    """

    def __init__(
        self,
        model: Model,
        update: Update[Msg, Model],
        subscriptions: Subscriptions[Msg, Model],
    ) -> None:
        self.message_queue: Queue = Queue()
        self.subscribers: Subscribers = Subscribers(self.message_queue.put)
        self.publishers: Publishers = Publishers()
        self.loop_thread: Thread = Thread(target=self.__loop__, daemon=True)

        self.model: Model = model
        self.update: Update[Msg, Model] = update
        self.subscriptions: Subscriptions[Msg, Model] = subscriptions

        self.loop_thread.start()

    def __loop__(self) -> None:
        while True:
            subs = self.subscriptions(self.model)
            self.subscribers.subscribe(subs)

            msg = self.message_queue.get()
            (new_model, cmd) = self.update(msg, self.model)

            if cmd is not None:
                self.publishers.publish(cmd)

            self.model = new_model

    @staticmethod
    def run(
        model: Model,
        update: Update[Msg, Model],
        subscriptions: Subscriptions[Msg, Model],
    ) -> None:
        """
        Run a ROS controller (Remember to run rospy.init_node before and rospy.
        spin after!)
        """
        Controller(model, update, subscriptions)
