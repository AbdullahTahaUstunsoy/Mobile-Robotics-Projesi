#!/usr/bin/env python3
"""Relative multi-waypoint corridor route with IMU heading and return home."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class MultiWaypoint(Node):

    def __init__(self):
        super().__init__('multi_waypoint_navigation')

        self.first_m = float(
            self.declare_parameter('first_m', 0.5).value
        )
        self.second_m = float(
            self.declare_parameter('second_m', 1.0).value
        )

        self.pose = None
        self.imu_yaw = None
        self.safety_stop = None
        self.last_seen = {}

        self.home = None
        self.initial_imu = None
        self.initial_odom_yaw = None

        self.targets = []
        self.index = 0
        self.mode = 'WAIT'
        self.done = False

        self.phase_since = time.monotonic()
        self.last_log = 0.0
        self.hold_until = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_raw', 10
        )

        self.create_subscription(
            Odometry, '/odom',
            self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, '/imu',
            self.on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Bool, '/safety/stop',
            self.on_safety, 10
        )

        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            'Coklu hedef navigasyonu hazir; sensorler bekleniyor.'
        )

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        self.pose = (
            pos.x,
            pos.y,
            yaw(msg.pose.pose.orientation)
        )
        self.last_seen['odom'] = time.monotonic()

    def on_imu(self, msg):
        self.imu_yaw = yaw(msg.orientation)
        self.last_seen['imu'] = time.monotonic()

    def on_safety(self, msg):
        self.safety_stop = msg.data
        self.last_seen['safety'] = time.monotonic()

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def finish(self, message):
        self.command()
        self.done = True
        self.get_logger().info(message)

    def imu_heading_in_odom(self):
        delta = wrap(
            self.imu_yaw - self.initial_imu
        )
        return wrap(
            self.initial_odom_yaw + delta
        )

    def turn_speed(self, error):
        if abs(error) <= math.radians(3):
            return 0.0

        speed = min(
            0.4,
            max(0.12, 1.2 * abs(error))
        )
        return math.copysign(speed, error)

    def activate_route(self, now):
        if not (
            0.1 <= self.first_m
            < self.second_m <= 1.5
        ):
            self.finish(
                'Gecersiz mesafe. '
                '0.1 <= first_m < second_m <= 1.5 olmali.'
            )
            return

        x, y, odom_yaw = self.pose

        self.home = (x, y)
        self.initial_imu = self.imu_yaw
        self.initial_odom_yaw = odom_yaw

        def point(distance):
            return (
                x + distance * math.cos(odom_yaw),
                y + distance * math.sin(odom_yaw)
            )

        # Ilk hedef, ikinci hedef, geri ara hedef, ev
        self.targets = [
            point(d) for d in (
                self.first_m,
                self.second_m,
                self.first_m,
                0.0
            )
        ]

        self.index = 0
        self.mode = 'DRIVE'
        self.phase_since = now

        self.get_logger().info(
            f'BASLANGIC: ({x:.2f}, {y:.2f}), '
            f'rota: {self.first_m:.2f} -> '
            f'{self.second_m:.2f} -> '
            f'{self.first_m:.2f} -> 0.00 m'
        )

    def tick(self):
        if self.done:
            return

        now = time.monotonic()

        # Sensorlerin gelmesini bekle
        if (
            self.pose is None
            or self.imu_yaw is None
            or self.safety_stop is None
        ):
            self.command()

            if now - self.phase_since > 30.0:
                self.finish(
                    'Sensorler gelmedi, gorev iptal.'
                )
            return

        # Eski sensor verisiyle hareket etme
        if any(
            now - self.last_seen.get(name, 0.0) > 5.0
            for name in ('odom', 'imu', 'safety')
        ):
            self.finish(
                'Sensor verisi eskidi; robot durduruldu.'
            )
            return

        # Bumper / LiDAR guvenlik durdurmasi
        if self.safety_stop:
            self.finish(
                'Guvenlik STOP! Gorev iptal, robot durduruldu.'
            )
            return

        if self.mode == 'WAIT':
            self.activate_route(now)
            return

        if now - self.phase_since > 120.0:
            self.finish(
                'Hedef icin 120 s zaman asimi; robot durduruldu.'
            )
            return

        # Her hedefte kisa sure dur
        if self.mode == 'HOLD':
            self.command()

            if now >= self.hold_until:
                self.mode = 'DRIVE'
                self.phase_since = now
            return

        # Son park: ilk baktigi yone don
        if self.mode == 'PARK':
            error = wrap(
                self.initial_imu - self.imu_yaw
            )

            if abs(error) <= math.radians(3):
                self.finish(
                    'GOREV TAMAMLANDI: 4 hedef, '
                    'eve donus ve park basarili!'
                )
                return

            self.command(
                angular=self.turn_speed(error)
            )

            if now - self.last_log >= 1.0:
                self.get_logger().info(
                    f'PARK | Kalan aci: '
                    f'{math.degrees(error):.1f} derece'
                )
                self.last_log = now
            return

        # Hedefe git
        x, y, _ = self.pose
        gx, gy = self.targets[self.index]

        dx = gx - x
        dy = gy - y

        remaining = math.hypot(dx, dy)

        if remaining <= 0.04:
            self.command()

            self.get_logger().info(
                f'HEDEF {self.index + 1}/4 TAMAM | '
                f'Hata: {remaining:.3f} m'
            )

            self.index += 1

            if self.index == len(self.targets):
                self.mode = 'PARK'
                self.get_logger().info(
                    'Baslangica donuldu; '
                    'park yonu ayarlaniyor.'
                )
            else:
                self.mode = 'HOLD'
                self.hold_until = now + 1.0

            self.phase_since = now
            return

        target_yaw = math.atan2(dy, dx)

        error = wrap(
            target_yaw - self.imu_heading_in_odom()
        )

        # Yon hatasi buyukse once yerinde don
        if abs(error) > math.radians(12):
            linear = 0.0
        elif remaining < 0.12:
            linear = 0.05
        else:
            linear = 0.08

        self.command(
            linear,
            self.turn_speed(error)
        )

        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f'HEDEF {self.index + 1}/4 | '
                f'Kalan: {remaining:.2f} m | '
                f'Yon hatasi: '
                f'{math.degrees(error):.1f} derece'
            )
            self.last_log = now


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = MultiWaypoint()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(
                node, timeout_sec=0.1
            )
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            node.command()
            time.sleep(0.05)

        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

