"""
TurtleBot3 commands.
"""

from geometry_msgs.msg import Twist, Vector3
from lib.controller import Cmd


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


def turn_with(vel_angular: float) -> Cmd:
    """
    Have TurtleBot turn with the given angular velocity.
    """
    return velocity(linear=0.0, angular=vel_angular)


def drive_with(vel_linear: float) -> Cmd:
    """
    Have TurtleBot drive with the given linear velocity.
    """
    return velocity(linear=vel_linear, angular=0.0)


"""
Instruct TurtleBot to stop moving.
"""
stop: Cmd = velocity(linear=0.0, angular=0.0)
