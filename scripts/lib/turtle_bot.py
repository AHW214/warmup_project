import rospy
from dataclasses import dataclass
from geometry_msgs.msg import Twist, Vector3
from lib.vector2 import Vector2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


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


def twist_from_velocities(velocity_angular: float, velocity_linear: float) -> Twist:
    return Twist(
        angular=Vector3(x=0.0, y=0.0, z=velocity_angular),
        linear=Vector3(x=velocity_linear, y=0.0, z=0.0),
    )


class TurtleBot:
    prev_time: float = 0.0
    publisher: rospy.Publisher = None
    subscriber: rospy.Subscriber = None

    def update(self, transform: Transform, delta_time: float) -> None:
        pass

    def receive(self, odom: Odometry) -> None:
        current_time = odom.header.stamp.to_sec()

        delta_time = current_time - self.prev_time
        transform = transform_from_odometry(odom)

        self.update(transform, delta_time)
        self.prev_time = current_time

    def send(self, velocity_angular: float, velocity_linear: float) -> None:
        twist = twist_from_velocities(velocity_angular, velocity_linear)
        self.publisher.publish(twist)

    def run(self) -> None:
        rospy.init_node("turtle_bot_client")

        self.publisher = rospy.Publisher(
            name="/cmd_vel", data_class=Twist, queue_size=10
        )

        self.subscriber = rospy.Subscriber(
            name="/odom", data_class=Odometry, callback=self.receive
        )

        rospy.spin()
