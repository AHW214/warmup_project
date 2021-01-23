from queue import Queue
from threading import Thread
from typing import Callable, Generic, List, Optional, Tuple, TypeVar
import rospy
from lib.controller.cmd import Cmd, Publishers
from lib.controller.sub import Sub, Subscribers


Model = TypeVar("Model")
Msg = TypeVar("Msg")

Update = Callable[[Msg, Model], Tuple[Model, Optional[Cmd]]]
Subscriptions = Callable[[Model], List[Sub[Msg]]]


class Controller(Generic[Model, Msg]):
    def __init__(self, model, update, subscriptions) -> None:
        self.message_queue: Queue = Queue()
        self.subscribers: Subscribers = Subscribers(self.message_queue.put)
        self.publishers: Publishers = Publishers()
        self.loop_thread: Thread = Thread(target=self.run, daemon=True)

        self.model: Model = model
        self.update: Update[Msg, Model] = update
        # should have two type vars
        self.subscriptions: Subscriptions[Model] = subscriptions

        rospy.init_node("robot_client")
        self.loop_thread.start()
        rospy.spin()

    def run(self) -> None:
        while True:
            new_subs = self.subscriptions(self.model)
            self.subscribers.update(new_subs)

            msg = self.message_queue.get()
            (new_model, cmd) = self.update(msg, self.model)
            self.publishers.update(cmd)
            self.model = new_model
