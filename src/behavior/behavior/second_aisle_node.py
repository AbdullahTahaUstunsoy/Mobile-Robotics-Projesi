#!/usr/bin/env python3
"""Navigate west along the second greenhouse aisle."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class SecondAisle(Node):

    def __init__(self):
        super().__init__('second_aisle_navigation')

        self.pose = None
        self.imu_yaw = None
        self.safety = None
        self.last_seen = {}

        self.start_imu = None
        self.start_odom_yaw = None
        self.target = None

        self.red_seen = False
        self.green_seen = False

        self.mode = 'WAIT'
        self.done = False
        self.start_time = time.monotonic()
        self.last_log = 0.0

        # Gazebo baslangici X=-0.5 oldugundan
        # odom X=-0.20, dunya X=-0.70 demektir.
        self.target_odom_x = -0.20

        self.publisher = self.create_publisher(
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

        self.create_subscription(
            PointStamped,
            '/color_detector/red_apple',
            self.on_red, 10
        )

        self.create_subscription(
            PointStamped,
            '/color_detector/green_apple',
            self.on_green, 10
        )

        self.create_timer(0.1, self.control)

        self.get_logger().info(
            'Ikinci koridor navigasyonu hazir.'
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position

        self.pose = (
            p.x, p.y,
            yaw(msg.pose.pose.orientation)
        )

        self.last_seen['odom'] = time.monotonic()

    def on_imu(self, msg):
        self.imu_yaw = yaw(msg.orientation)
        self.last_seen['imu'] = time.monotonic()

    def on_safety(self, msg):
        self.safety = msg.data
        self.last_seen['safety'] = time.monotonic()

    def on_red(self, msg):
        if self.mode == 'DRIVE' and not self.red_seen:
            self.red_seen = True
            self.get_logger().info(
                'IKINCI KORIDOR: KIRMIZI ELMA ALGILANDI!'
            )

    def on_green(self, msg):
        if self.mode == 'DRIVE' and not self.green_seen:
            self.green_seen = True
            self.get_logger().info(
                'IKINCI KORIDOR: YESIL ELMA ALGILANDI!'
            )

    def command(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.publisher.publish(cmd)

    def finish(self, message):
        self.command()
        self.done = True
        self.get_logger().info(message)

    def heading(self):
        delta = wrap(self.imu_yaw - self.start_imu)
        return wrap(self.start_odom_yaw + delta)

    def control(self):
        if self.done:
            return

        now = time.monotonic()

        if (
            self.pose is None
            or self.imu_yaw is None
            or self.safety is None
        ):
            self.command()

            if now - self.start_time > 30.0:
                self.finish('Sensorler gelmedi. Iptal.')
            return

        if any(
            now - self.last_seen.get(k, 0) > 5.0
            for k in ('odom', 'imu', 'safety')
        ):
            self.finish('Sensor verisi eskidi. STOP.')
            return

        if self.safety:
            self.finish('Guvenlik STOP! Gorev iptal.')
            return

        if self.mode == 'WAIT':
            x, y, angle = self.pose

            # Yalnizca test edilmis koridor
            # gecisinden sonraki konumdan baslat.
            if not (
                6.8 <= x <= 7.7
                and -1.55 <= y <= -1.05
                and abs(wrap(angle - math.pi))
                    <= math.radians(15)
            ):
                self.finish(
                    'Ikinci koridor baslangic '
                    'konumu uygun degil. Iptal.'
                )
                return

            self.start_imu = self.imu_yaw
            self.start_odom_yaw = angle

            # Mevcut koridorun Y degerini koru.
            self.target = (
                self.target_odom_x,
                y
            )

            self.mode = 'DRIVE'
            self.start_time = now

            self.get_logger().info(
                f'BASLANGIC: ({x:.2f}, {y:.2f})'
            )
            self.get_logger().info(
                f'HEDEF: ({self.target[0]:.2f}, '
                f'{self.target[1]:.2f})'
            )
            return

        if now - self.start_time > 900.0:
            self.finish('Gorev zaman asimi. STOP.')
            return

        x, y, _ = self.pose
        tx, ty = self.target

        dx = tx - x
        dy = ty - y
        remaining = math.hypot(dx, dy)

        if remaining <= 0.05:
            self.finish(
                'IKINCI KORIDOR TAMAMLANDI! '
                f'Kirmizi elma: {self.red_seen}, '
                f'Yesil elma: {self.green_seen}. '
                'Robot acik alanda durdu.'
            )
            return

        target_heading = math.atan2(dy, dx)
        error = wrap(target_heading - self.heading())

        if abs(error) > math.radians(12):
            linear = 0.0
        elif remaining < 0.15:
            linear = 0.05
        else:
            linear = 0.08

        angular = 0.0

        if abs(error) > math.radians(3):
            angular = math.copysign(
                min(0.30, max(0.12, 1.2 * abs(error))),
                error
            )

        self.command(linear, angular)

        if now - self.last_log >= 3.0:
            self.get_logger().info(
                f'Kalan: {remaining:.2f} m | '
                f'Yon hatasi: '
                f'{math.degrees(error):.1f} derece'
            )
            self.last_log = now


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = SecondAisle()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
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
