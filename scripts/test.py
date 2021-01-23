#!/usr/bin/env python3

from dataclasses import dataclass, replace
from typing import List, Optional, Tuple, Union
from lib.controller.controller import Cmd, Controller, Sub
import lib.turtle_bot_2 as turtle


@dataclass
class Model:
    thing: str


@dataclass
class Odom:
    transform: turtle.Transform


Msg = Union[Odom]


init_model: Model = Model("")


def update(msg: Msg, model: Model) -> Tuple[Model, Optional[Cmd]]:
    if isinstance(msg, Odom):
        print("odom")
        new_model = replace(model, thing="yay")
        return (new_model, None)

    return (model, None)


def subscriptions(_: Model) -> List[Sub[Msg]]:
    return [turtle.odometry(Odom)]


if __name__ == "__main__":
    Controller(model=init_model, update=update, subscriptions=subscriptions)
