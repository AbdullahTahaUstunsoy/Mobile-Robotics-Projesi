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

from behavior.navigation_node import (
    yaw_from_quaternion,
    normalize_angle,
)


class LRoute(Node):

    def __init__(self):
        super().__init__('l_route')

        self.distance = 0.30
        self.turn_angle = math.pi / 2

        self.pose = None
        self.yaw = None
        self.safety_stop = None

        self.last_odom = 0.0
        self.last_imu = 0.0
        self.last_safety = 0.0

        self.state = 'WAIT'
        self.done = False
        self.wait_start = time.monotonic()
        self.phase_start = None
        self.last_log = 0.0

        self.leg_start = None
        self.heading_target = None
        self.turn_target = None

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

        self.create_timer(0.1, self.control)

        self.get_logger().info(
            'L rota sistemi hazir. Sensorler bekleniyor.'
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)
        self.last_odom = time.monotonic()

    def on_imu(self, msg):
        self.yaw = yaw_from_quaternion(
            msg.orientation
        )
        self.last_imu = time.monotonic()

    def on_safety(self, msg):
        self.safety_stop = msg.data
        self.last_safety = time.monotonic()

    def publish_speed(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.publish_speed()

    def finish(self, message):
        self.stop()
        self.done = True
        self.get_logger().info(message)

    def start_drive(self, state, now):
        self.state = state
        self.leg_start = self.pose
        self.heading_target = self.yaw
        self.phase_start = now

        self.stop()
        self.get_logger().info(
            f'{state}: 30 cm ilerleme basladi.'
        )

    def start_turn(self, now):
        self.state = 'TURN'
        self.turn_target = normalize_angle(
            self.yaw + self.turn_angle
        )
        self.phase_start = now

        self.stop()
        self.get_logger().info(
            '90 derece sola donus basladi.'
        )

    def control(self):
        if self.done:
            return

        now = time.monotonic()

        # Ilk veriler gelmeden hareket etme.
        if (self.pose is None or
                self.yaw is None or
                self.safety_stop is None):
            self.stop()
            if now - self.wait_start > 30.0:
                self.finish('Sensor bekleme suresi asildi.')
            return

        # Eski sensor verisiyle hareket etme.
        if (now - self.last_odom > 5.0 or
                now - self.last_imu > 5.0 or
                now - self.last_safety > 5.0):
            self.finish('Sensor verisi kesildi. Duruldu.')
            return

        # Arkadaslarin guvenlik sistemi oncelikli.
        if self.safety_stop:
            self.finish('Guvenlik uyarisi! Gorev iptal edildi.')
            return

        if self.state == 'WAIT':
            self.start_drive('DRIVE1', now)
            return

        # Her asama icin ust sure.
        if now - self.phase_start > 120.0:
            self.finish('Asama suresi asildi. Duruldu.')
            return

        if self.state in ('DRIVE1', 'DRIVE2'):

            x, y = self.pose
            sx, sy = self.leg_start

            traveled = math.hypot(x - sx, y - sy)
            remaining = self.distance - traveled

            if remaining <= 0.04:
                self.stop()

                if self.state == 'DRIVE1':
                    self.start_turn(now)
                else:
                    self.finish(
                        'L ROTA TAMAMLANDI! Robot durdu.'
                    )
                return

            error = normalize_angle(
                self.heading_target - self.yaw
            )

            angular = max(
                -0.25,
                min(0.25, 1.2 * error)
            )

            if abs(error) > 0.35:
                linear = 0.0
            elif remaining < 0.12:
                linear = 0.05
            else:
                linear = 0.08

            self.publish_speed(linear, angular)

            if now - self.last_log >= 1.0:
                self.get_logger().info(
                    f'{self.state}: '
                    f'Kalan {remaining:.2f} m'
                )
                self.last_log = now

        elif self.state == 'TURN':

            error = normalize_angle(
                self.turn_target - self.yaw
            )

            if abs(error) <= math.radians(3):
                self.stop()
                self.get_logger().info(
                    '90 derece donus tamamlandi.'
                )
                self.start_drive('DRIVE2', now)
                return

            speed = min(
                0.45,
                max(0.12, 1.2 * abs(error))
            )

            angular = math.copysign(speed, error)
            self.publish_speed(0.0, angular)

            if now - self.last_log >= 1.0:
                self.get_logger().info(
                    f'TURN: Kalan aci '
                    f'{math.degrees(error):.1f} derece'
                )
                self.last_log = now


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = LRoute()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        for _ in range(3):
            node.stop()
            time.sleep(0.05)

        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
