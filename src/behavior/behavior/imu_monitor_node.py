#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuMonitor(Node):

    def __init__(self):
        super().__init__('imu_monitor')

        self.last_log = 0.0

        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'IMU takip sistemi baslatildi.'
        )

    def imu_callback(self, msg):
        q = msg.orientation

        # Quaternion -> Yaw (radyan)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)

        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        # Radyan -> Derece
        yaw_deg = math.degrees(yaw_rad)

        # Z eksenindeki donus hizi
        angular_velocity = msg.angular_velocity.z

        now = time.monotonic()

        # Terminale saniyede bir yazdir
        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f'Yon: {yaw_deg:.2f} derece | '
                f'Donus hizi: {angular_velocity:.3f} rad/s'
            )
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)

    node = ImuMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
