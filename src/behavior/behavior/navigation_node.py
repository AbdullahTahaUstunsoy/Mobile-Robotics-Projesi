#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Navigation(Node):

    def __init__(self):
        super().__init__('navigation')

        self.pose = None
        self.imu_yaw = None
        self.safety_stop = None

        self.last_odom = 0.0
        self.last_imu = 0.0
        self.last_safety = 0.0

        self.target = None
        self.start_yaw = None
        self.start_time = None
        self.last_log = 0.0
        self.done = False

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel_raw', 10
        )

        self.create_subscription(
            Odometry, '/odom', self.on_odom,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Imu, '/imu', self.on_imu,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Bool, '/safety/stop', self.on_safety, 10
        )

        self.create_timer(0.1, self.control)

        self.get_logger().info(
            'Navigasyon hazir. Sensorler bekleniyor.'
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        self.pose = (p.x, p.y, yaw)
        self.last_odom = time.monotonic()

    def on_imu(self, msg):
        self.imu_yaw = yaw_from_quaternion(
            msg.orientation
        )
        self.last_imu = time.monotonic()

    def on_safety(self, msg):
        self.safety_stop = msg.data
        self.last_safety = time.monotonic()

    def stop(self):
        self.publisher.publish(Twist())

    def finish(self, message):
        self.stop()
        self.done = True
        self.get_logger().info(message)

    def control(self):
        if self.done:
            return

        now = time.monotonic()

        if (self.pose is None or
                self.imu_yaw is None or
                self.safety_stop is None):
            self.stop()
            return

        # Sensor verileri guncel olmali.
        if (now - self.last_odom > 4.0 or
                now - self.last_imu > 4.0 or
                now - self.last_safety > 4.0):
            self.stop()
            if self.start_time is not None:
                self.finish('Sensor verisi kesildi. Gorev durduruldu.')
            return

        # LiDAR veya bumper guvenlik uyarisi.
        if self.safety_stop:
            self.stop()
            if self.start_time is not None:
                self.finish('Guvenlik uyarisi! Navigasyon durduruldu.')
            return

        x, y, odom_yaw = self.pose

        # Ilk konumdan 30 cm ileriye hedef belirle.
        if self.target is None:
            distance = 0.30

            target_x = x + distance * math.cos(odom_yaw)
            target_y = y + distance * math.sin(odom_yaw)

            self.target = (target_x, target_y)
            self.start_yaw = self.imu_yaw
            self.start_time = now

            self.get_logger().info(
                f'Hedef: x={target_x:.2f}, y={target_y:.2f}'
            )

        # VirtualBox yavasligina karsi ust sure.
        if now - self.start_time > 120.0:
            self.finish('Sure asimi! Robot durduruldu.')
            return

        dx = self.target[0] - x
        dy = self.target[1] - y
        remaining = math.hypot(dx, dy)

        if remaining <= 0.04:
            self.finish('HEDEFE ULASILDI! Robot durdu.')
            return

        # IMU ile baslangic yonunu koru.
        heading_error = normalize_angle(
            self.start_yaw - self.imu_yaw
        )

        cmd = Twist()

        cmd.angular.z = max(
            -0.25, min(0.25, 1.2 * heading_error)
        )

        # Yon hatasi buyukse once duzelt.
        if abs(heading_error) > 0.35:
            cmd.linear.x = 0.0
        elif remaining < 0.12:
            cmd.linear.x = 0.05
        else:
            cmd.linear.x = 0.08

        self.publisher.publish(cmd)

        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f'Kalan mesafe: {remaining:.2f} m | '
                f'IMU yon: {math.degrees(self.imu_yaw):.1f} derece'
            )
            self.last_log = now


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = Navigation()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Gazebo son hiz komutunu koruyabildigi
        # icin kapanirken sifir hiz yayinla.
        for _ in range(3):
            node.stop()
            time.sleep(0.05)

        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
