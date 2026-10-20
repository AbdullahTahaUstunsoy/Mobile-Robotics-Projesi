#!/usr/bin/env python3
"""First greenhouse aisle: IMU navigation and apple detection."""

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


class GreenhouseRoute(Node):

    def __init__(self):
        super().__init__('greenhouse_route')

        # Gazebo world X koordinati.
        # Ilk testte 2.0, tam koridorda 6.8 kullanacagiz.
        self.end_x = float(
            self.declare_parameter('end_x', 2.0).value
        )

        self.pose = None
        self.imu_yaw = None
        self.safety_stop = None
        self.last_seen = {}

        self.start_imu = None
        self.start_odom_yaw = None
        self.target = None

        self.red_seen = False
        self.green_seen = False

        self.started = time.monotonic()
        self.last_log = 0.0
        self.mode = 'WAIT'
        self.done = False

        self.pub = self.create_publisher(
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
            self.on_red_apple, 10
        )
        self.create_subscription(
            PointStamped,
            '/color_detector/green_apple',
            self.on_green_apple, 10
        )

        self.create_timer(0.1, self.control)

        self.get_logger().info(
            'Sera navigasyonu hazir.'
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
        self.safety_stop = msg.data
        self.last_seen['safety'] = time.monotonic()

    def on_red_apple(self, msg):
        if self.mode == 'DRIVE' and not self.red_seen:
            self.red_seen = True
            self.get_logger().info(
                'KIRMIZI ELMA ALGILANDI! '
                f'Kamera x={msg.point.x:.2f}, '
                f'alan={msg.point.z:.4f}'
            )

    def on_green_apple(self, msg):
        if self.mode == 'DRIVE' and not self.green_seen:
            self.green_seen = True
            self.get_logger().info(
                'YESIL ELMA ALGILANDI!'
            )

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def finish(self, message):
        self.command()
        self.done = True
        self.get_logger().info(message)

    def heading(self):
        delta = wrap(
            self.imu_yaw - self.start_imu
        )
        return wrap(
            self.start_odom_yaw + delta
        )

    def control(self):
        if self.done:
            return

        now = time.monotonic()

        if (
            self.pose is None
            or self.imu_yaw is None
            or self.safety_stop is None
        ):
            self.command()
            if now - self.started > 30:
                self.finish('Sensorler gelmedi.')
            return

        if any(
            now - self.last_seen.get(k, 0) > 5.0
            for k in ('odom', 'imu', 'safety')
        ):
            self.finish('Sensor verisi eskidi. STOP.')
            return

        if self.safety_stop:
            self.finish('Guvenlik STOP! Gorev iptal.')
            return

        if self.mode == 'WAIT':
            # Bu surum taze Gazebo baslangici gerektirir.
            x, y, a = self.pose

            if (
                abs(x) > 0.10
                or abs(y) > 0.10
                or abs(wrap(a)) > math.radians(10)
            ):
                self.finish(
                    'Baslangic odometrisi sifir degil. '
                    'Gazebo yeniden baslatilmali.'
                )
                return

            # Gazebo baslangic X=-0.5, Y=0.65
            distance = self.end_x - (-0.5)

            if not 0.2 <= distance <= 7.5:
                self.finish('Gecersiz end_x.')
                return

            self.start_imu = self.imu_yaw
            self.start_odom_yaw = a

            # Koridor ekseni sabit: odom Y=0.
            # Gazebo X=6.8 -> odom X=7.3.
            # Baslangic acisi hedefi kaydirmamali.
            self.target = (distance, 0.0)

            self.mode = 'DRIVE'
            self.started = now

            self.get_logger().info(
                f'HEDEF: Gazebo X={self.end_x:.2f} | '
                f'Mesafe={distance:.2f} m'
            )
            return

        # VirtualBox yavasligi icin uzun ust sinir.
        if now - self.started > 900:
            self.finish('Gorev zaman asimi. STOP.')
            return

        x, y, _ = self.pose
        remaining = self.target[0] - x

        # Koridor merkezinden fazla uzaklasirsak
        # guvenlik sistemini beklemeden dur.
        if abs(y) > 0.12:
            self.finish(
                f'Yanal sapma cok fazla: {y:.3f} m. '
                'Navigasyon durduruldu.'
            )
            return

        if remaining <= 0.05:
            self.finish(
                'ILK KORIDOR HEDEFI TAMAMLANDI! '
                f'Kirmizi elma: {self.red_seen}, '
                f'Yesil elma: {self.green_seen}'
            )
            return

        # Nominal yon 0 derece.
        # Y sapmasina gore kucuk duzeltme yap.
        target_yaw = math.atan2(-2.0 * y, 1.0)
        target_yaw = max(
            -math.radians(10),
            min(math.radians(10), target_yaw)
        )

        error = wrap(target_yaw - self.heading())

        angular = 0.0

        if abs(error) > math.radians(1.0):
            angular = math.copysign(
                min(0.25, max(0.10, 1.5 * abs(error))),
                error
            )

        if abs(error) > math.radians(12):
            linear = 0.0
        elif remaining < 0.15:
            linear = 0.05
        else:
            linear = 0.08

        self.command(linear, angular)

        if now - self.last_log >= 2.0:
            self.get_logger().info(
                f'Kalan: {remaining:.2f} m | Y sapmasi: {y:.3f} m | '
                f'Yon hatasi: '
                f'{math.degrees(error):.1f} derece'
            )
            self.last_log = now


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = GreenhouseRoute()

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
