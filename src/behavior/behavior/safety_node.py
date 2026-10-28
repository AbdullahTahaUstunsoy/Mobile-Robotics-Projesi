"""Filter velocity commands with the bumper and the lidar obstacle state.

Subscribes:  /cmd_vel_raw (geometry_msgs/Twist): commands from teleop / navigation
             /bumper (ros_gz_interfaces/Contacts): published by Gazebo only while the bumper touches something
             /obstacle_detector/state (std_msgs/String): CLEAR, WARN or STOP
Publishes:   /cmd_vel (geometry_msgs/Twist): the safe command, forward motion removed while stopped
             /safety/stop (std_msgs/Bool): True while forward motion is blocked

The last command is re-checked on a timer: DiffDrive keeps driving with the last command it got,
so a stop has to be sent even when no new command arrives.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, String

from behavior.safety import CLEAR, SafetyParams, filter_cmd, stop_reason


class Safety(Node):
    def __init__(self):
        super().__init__("safety")
        defaults = SafetyParams()
        self.params = SafetyParams(
            **{
                name: type(value)(self.declare_parameter(name, value).value)
                for name, value in vars(defaults).items()
            }
        )
        rate = self.declare_parameter("rate", 20.0).value
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.stop_pub = self.create_publisher(Bool, "safety/stop", 10)
        self.create_subscription(Twist, "cmd_vel_raw", self.on_cmd, 10)
        self.create_subscription(Contacts, "bumper", self.on_bumper, 10)
        self.create_subscription(String, "obstacle_detector/state", self.on_obstacle_state, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.cmd = Twist()
        self.sent = None  # (linear, angular) last published on cmd_vel
        self.last_contact = None  # seconds, node clock
        self.obstacle_state = None
        self.reason = CLEAR

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.cmd = msg
        self.update(force=True)

    def on_bumper(self, msg):
        if msg.contacts:
            self.last_contact = self.now()
            self.update()

    def on_obstacle_state(self, msg):
        self.obstacle_state = msg.data

    def on_timer(self):
        self.update()
        self.stop_pub.publish(Bool(data=self.reason != CLEAR))

    def update(self, force=False):
        reason = stop_reason(self.last_contact, self.now(), self.obstacle_state, self.params)
        if reason != self.reason:
            # One call site per severity: rclpy refuses to log from the same line with a different one
            if reason == CLEAR:
                self.get_logger().info(f"{reason}: forward motion allowed")
            else:
                self.get_logger().warning(f"{reason}: forward motion blocked")
            self.reason = reason

        linear, angular = filter_cmd(self.cmd.linear.x, self.cmd.angular.z, reason)
        # Every new command is passed on; otherwise publish only when the filter changes the output
        if force or (linear, angular) != self.sent:
            out = Twist()
            out.linear.x, out.angular.z = linear, angular
            self.cmd_pub.publish(out)
            self.sent = (linear, angular)


def main():
    rclpy.init()
    node = Safety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
