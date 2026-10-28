"""Run the greenhouse mission (config/mission.yaml): both aisles, checkpoint scan, return home and park.

Subscribes:  /odom (nav_msgs/Odometry): position, and the heading at the mission start
             /imu (sensor_msgs/Imu): heading changes (heading = start odom yaw + IMU yaw change)
             /safety/stop (std_msgs/Bool): driving pauses while True
             /color_detector/<label> (geometry_msgs/PointStamped): apples and checkpoints, logged per stage
Publishes:   /cmd_vel_raw (geometry_msgs/Twist), filtered by the safety node

Needs a fresh simulation (odom at its origin). The spawn_x / spawn_y / spawn_yaw parameters must match
the spawn pose in sim.launch.py; sim.launch.py passes them when started with mission:=true.
Exits with code 0 when the mission is complete, 1 when it fails.
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import yaml

from behavior.color_detection import LABELS
from behavior.mission import DONE, FAILED, load_mission, Mission, RUNNING
from behavior.navigation import wrap, yaw_from_quaternion


class MissionNode(Node):

    def __init__(self):
        super().__init__('greenhouse_mission')
        default_file = os.path.join(get_package_share_directory('behavior'), 'config', 'mission.yaml')
        mission_file = self.declare_parameter('mission_file', default_file).value
        spawn = tuple(float(self.declare_parameter(name, value).value)
                      for name, value in (('spawn_x', -0.5), ('spawn_y', 0.65), ('spawn_yaw', 0.0)))
        self.sensor_wait = float(self.declare_parameter('sensor_wait', 60.0).value)
        self.sensor_timeout = float(self.declare_parameter('sensor_timeout', 5.0).value)
        safety_timeout = float(self.declare_parameter('safety_timeout', 30.0).value)
        self.log_period = float(self.declare_parameter('log_period', 2.0).value)

        with open(mission_file) as f:
            self.start_check, stages = load_mission(yaml.safe_load(f), spawn)
        self.mission = Mission(stages, safety_timeout)

        self.pose = None            # odom (x, y, yaw)
        self.imu_yaw = None
        self.safety_stop = None
        self.last_seen = {}
        self.heading_ref = None     # (odom yaw, imu yaw) at the mission start
        self.created = time.monotonic()
        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, 'safety/stop', self.on_safety, 10)
        for label in LABELS:
            self.create_subscription(PointStamped, f'color_detector/{label}',
                                     lambda _, label=label: self.mission.on_detection(label), 10)
        self.create_timer(0.1, self.control)
        self.get_logger().info(f'Mission loaded: {len(stages)} stages from {mission_file}')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.last_seen['odom'] = time.monotonic()

    def on_imu(self, msg):
        q = msg.orientation
        self.imu_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.last_seen['imu'] = time.monotonic()

    def on_safety(self, msg):
        self.safety_stop = msg.data
        self.last_seen['safety'] = time.monotonic()

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def control(self):
        if self.mission.status != RUNNING:
            return
        now = time.monotonic()

        if self.pose is None or self.imu_yaw is None or self.safety_stop is None:
            if now - self.created > self.sensor_wait:
                self.mission.fail(f'no odom / imu / safety data after {self.sensor_wait:.0f} s')
            self.flush_events()
            return
        stale = [k for k in ('odom', 'imu', 'safety') if now - self.last_seen[k] > self.sensor_timeout]
        if stale:
            self.mission.fail(f'stale sensor data: {", ".join(stale)}')
            self.flush_events()
            return

        if self.heading_ref is None:
            # Only this node may drive; a running teleop would fight the mission
            if self.count_publishers('cmd_vel_raw') > 1:
                self.mission.fail('another node is publishing cmd_vel_raw')
            elif not self.start_check.ok(self.pose):
                x, y, a = self.pose
                self.mission.fail(f'odom is not at its origin ({x:.2f}, {y:.2f}, {a:.2f} rad), '
                                  'restart the simulation')
            else:
                self.heading_ref = (self.pose[2], self.imu_yaw)
            self.flush_events()
            if self.heading_ref is None:
                return

        heading = wrap(self.heading_ref[0] + wrap(self.imu_yaw - self.heading_ref[1]))
        linear, angular = self.mission.step(self.pose, heading, self.safety_stop, now)
        self.publish(linear, angular)
        self.flush_events()

        if self.mission.status == RUNNING and now - self.last_log >= self.log_period:
            self.get_logger().info(self.mission.progress)
            self.last_log = now

    def flush_events(self):
        for event in self.mission.events():
            if self.mission.status == FAILED and event.startswith('MISSION FAILED'):
                self.get_logger().error(event)
            else:
                self.get_logger().info(event)


def main():
    # Handle Ctrl+C ourselves so a stop command can still be published before shutdown
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MissionNode()
    try:
        while rclpy.ok() and node.mission.status == RUNNING:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.mission.fail('interrupted')
    finally:
        for _ in range(3):
            node.publish()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if node.mission.status == DONE else 1)


if __name__ == '__main__':
    main()
