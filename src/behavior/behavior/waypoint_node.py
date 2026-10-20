#!/usr/bin/env python3
"""Short odometry-based round trip with IMU heading and safety."""

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


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint_navigation')

        self.distance = self.declare_parameter(
            'forward_m', 0.20
        ).value

        self.pose = None
        self.imu = None
        self.safety = None
        self.timestamps = {}
        self.mode = 'WAIT'
        self.done = False
        self.home = None
        self.goal = None
        self.initial_imu = None
        self.initial_odom_yaw = None
        self.phase_time = time.monotonic()
        self.last_log = 0.0

        self.pub = self.create_publisher(
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
            'Waypoint navigasyon hazir.'
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (
            p.x, p.y,
            yaw(msg.pose.pose.orientation)
        )
        self.timestamps['odom'] = time.monotonic()

    def on_imu(self, msg):
        self.imu = yaw(msg.orientation)
        self.timestamps['imu'] = time.monotonic()

    def on_safety(self, msg):
        self.safety = msg.data
        self.timestamps['safety'] = time.monotonic()

    def publish(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.pub.publish(cmd)

    def finish(self, message):
        self.publish()
        self.done = True
        self.get_logger().info(message)

    def heading(self):
        # IMU'dan olculen donusu baslangic
        # odometri yonune gore hesapla.
        delta = wrap(self.imu - self.initial_imu)
        return wrap(self.initial_odom_yaw + delta)

    def control(self):
        if self.done:
            return

        now = time.monotonic()

        if (self.pose is None or self.imu is None
                or self.safety is None):
            self.publish()
            if now - self.phase_time > 30:
                self.finish('Sensorler gelmedi. Gorev iptal.')
            return

        if any(
            now - self.timestamps.get(topic, 0) > 5.0
            for topic in ('odom', 'imu', 'safety')
        ):
            self.finish('Sensor verisi eskidi. Duruldu.')
            return

        if self.safety:
            self.finish('Guvenlik uyarisi! Duruldu.')
            return

        if self.mode == 'WAIT':
            if not 0.10 <= self.distance <= 0.50:
                self.finish('forward_m 0.10-0.50 olmali.')
                return

            x, y, odom_yaw = self.pose
            self.home = (x, y)
            self.initial_imu = self.imu
            self.initial_odom_yaw = odom_yaw

            self.goal = (
                x + self.distance * math.cos(odom_yaw),
                y + self.distance * math.sin(odom_yaw)
            )

            self.mode = 'OUT'
            self.phase_time = now
            self.get_logger().info(
                f'HEDEF: {self.goal} | '
                f'BASLANGIC: {self.home}'
            )
            return

        if now - self.phase_time > 120:
            self.finish('Asama zaman asimi. Duruldu.')
            return

        if self.mode in ('OUT', 'RETURN'):
            target = (
                self.goal if self.mode == 'OUT'
                else self.home
            )

            x, y, _ = self.pose
            dx = target[0] - x
            dy = target[1] - y
            remaining = math.hypot(dx, dy)

            if remaining <= 0.04:
                self.publish()

                if self.mode == 'OUT':
                    self.mode = 'RETURN'
                    self.get_logger().info(
                        'Ilk hedef tamam. Eve donuluyor.'
                    )
                else:
                    self.mode = 'PARK'
                    self.get_logger().info(
                        'Baslangica gelindi. Yon duzeltiliyor.'
                    )

                self.phase_time = now
                return

            target_yaw = math.atan2(dy, dx)
            error = wrap(target_yaw - self.heading())

            angular = math.copysign(
                min(0.40, max(0.12, abs(error) * 1.2)),
                error
            ) if abs(error) > math.radians(3) else 0.0

            if abs(error) > math.radians(12):
                linear = 0.0
            elif remaining < 0.12:
                linear = 0.05
            else:
                linear = 0.08

            self.publish(linear, angular)

            if now - self.last_log >= 1.0:
                self.get_logger().info(
                    f'{self.mode} | '
                    f'Kalan {remaining:.2f} m | '
                    f'Yon hatasi {math.degrees(error):.1f} derece'
                )
                self.last_log = now

        elif self.mode == 'PARK':
            error = wrap(self.initial_imu - self.imu)

            if abs(error) <= math.radians(3):
                self.finish(
                    'GOREV TAMAMLANDI! Baslangica donuldu '
                    've park edildi.'
                )
                return

            angular = math.copysign(
                min(0.40, max(0.12, abs(error) * 1.2)),
                error
            )
            self.publish(0.0, angular)

            if now - self.last_log >= 1.0:
                self.get_logger().info(
                    f'PARK | Kalan aci: '
                    f'{math.degrees(error):.1f} derece'
                )
                self.last_log = now


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = Waypoint()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.publish()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
