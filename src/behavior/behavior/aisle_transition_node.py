#!/usr/bin/env python3
"""Move from first aisle end to second aisle using IMU and odometry."""

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


def get_yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class AisleTransition(Node):

    def __init__(self):
        super().__init__('aisle_transition')

        self.pose = None
        self.imu_yaw = None
        self.safety_stop = None
        self.last_seen = {}

        self.initial_imu = None
        self.initial_odom_yaw = None
        self.target = None

        self.red_checkpoint = False
        self.green_checkpoint = False

        self.mode = 'WAIT'
        self.done = False
        self.phase_start = time.monotonic()
        self.last_log = 0.0

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
            PointStamped, '/color_detector/red_checkpoint',
            self.on_red_checkpoint, 10
        )
        self.create_subscription(
            PointStamped, '/color_detector/green_checkpoint',
            self.on_green_checkpoint, 10
        )

        self.create_timer(0.1, self.control)

        self.get_logger().info(
            'Koridor gecis sistemi hazir.'
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (
            p.x, p.y,
            get_yaw(msg.pose.pose.orientation)
        )
        self.last_seen['odom'] = time.monotonic()

    def on_imu(self, msg):
        self.imu_yaw = get_yaw(msg.orientation)
        self.last_seen['imu'] = time.monotonic()

    def on_safety(self, msg):
        self.safety_stop = msg.data
        self.last_seen['safety'] = time.monotonic()

    def on_red_checkpoint(self, msg):
        if self.mode == 'SCAN' and not self.red_checkpoint:
            self.red_checkpoint = True
            self.get_logger().info(
                'RED CHECKPOINT ALGILANDI!'
            )

    def on_green_checkpoint(self, msg):
        if self.mode == 'SCAN' and not self.green_checkpoint:
            self.green_checkpoint = True
            self.get_logger().info(
                'GREEN CHECKPOINT ALGILANDI!'
            )

    def command(self, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.pub.publish(cmd)

    def finish(self, message):
        self.command()
        self.done = True
        self.get_logger().info(message)

    def heading(self):
        delta = wrap(self.imu_yaw - self.initial_imu)
        return wrap(self.initial_odom_yaw + delta)

    def turn(self, target_angle):
        error = wrap(target_angle - self.heading())

        if abs(error) <= math.radians(3):
            self.command()
            return True

        speed = min(0.35, max(0.12, abs(error) * 1.2))
        self.command(
            angular=math.copysign(speed, error)
        )

        now = time.monotonic()

        if now - self.last_log >= 2.0:
            self.get_logger().info(
                f'{self.mode} | Kalan aci: '
                f'{math.degrees(error):.1f} derece'
            )
            self.last_log = now

        return False

    def change_mode(self, mode):
        self.command()
        self.mode = mode
        self.phase_start = time.monotonic()
        self.get_logger().info(f'Yeni asama: {mode}')

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

            if now - self.phase_start > 30:
                self.finish('Sensorler gelmedi. Iptal.')
            return

        if any(
            now - self.last_seen.get(k, 0) > 5.0
            for k in ('odom', 'imu', 'safety')
        ):
            self.finish('Sensor verisi eskidi. STOP.')
            return

        if self.safety_stop:
            self.finish('Guvenlik STOP! Gecis iptal.')
            return

        if self.mode == 'WAIT':
            x, y, angle = self.pose

            # Yalnizca ilk koridor sonundaki,
            # checkpoint duvarina donuk konumdan basla.
            if not (
                6.8 <= x <= 7.7
                and abs(y) < 0.25
                and abs(wrap(angle - math.pi / 2))
                    < math.radians(15)
            ):
                self.finish(
                    'Robot beklenen koridor sonu '
                    'konumunda degil. Iptal.'
                )
                return

            self.initial_imu = self.imu_yaw
            self.initial_odom_yaw = angle

            # Dunya koordinatinda Y=+0.65
            # noktasindan Y=-0.65 noktasina.
            self.target = (x, y - 1.30)

            self.get_logger().info(
                f'Baslangic odom: ({x:.2f}, {y:.2f}) '
                f'hedef: ({self.target[0]:.2f}, '
                f'{self.target[1]:.2f})'
            )

            self.change_mode('SCAN')
            return

        if now - self.phase_start > 240.0:
            self.finish('Asama zaman asimi. STOP.')
            return

        if self.mode == 'SCAN':
            self.command()

            if now - self.phase_start >= 5.0:
                self.get_logger().info(
                    f'Checkpoint tarama: '
                    f'RED={self.red_checkpoint}, '
                    f'GREEN={self.green_checkpoint}'
                )

                # Mevcut +90 dereceden
                # -90 dereceye saga don.
                self.change_mode('TURN_SOUTH')
            return

        if self.mode == 'TURN_SOUTH':
            if self.turn(-math.pi / 2):
                self.change_mode('CROSS')
            return

        if self.mode == 'CROSS':
            x, y, _ = self.pose
            tx, ty = self.target

            dx = tx - x
            dy = ty - y
            remaining = math.hypot(dx, dy)

            if remaining <= 0.05:
                self.change_mode('TURN_WEST')
                return

            target_heading = math.atan2(dy, dx)
            error = wrap(target_heading - self.heading())

            if abs(error) > math.radians(10):
                linear = 0.0
            elif remaining < 0.15:
                linear = 0.05
            else:
                linear = 0.08

            angular = 0.0

            if abs(error) > math.radians(3):
                angular = math.copysign(
                    min(0.30, max(0.12, abs(error))),
                    error
                )

            self.command(linear, angular)

            if now - self.last_log >= 2.0:
                self.get_logger().info(
                    f'CROSS | Kalan: {remaining:.2f} m '
                    f'| Yon hatasi: '
                    f'{math.degrees(error):.1f} derece'
                )
                self.last_log = now
            return

        if self.mode == 'TURN_WEST':
            if self.turn(-math.pi):
                self.finish(
                    'KORIDOR GECISI TAMAMLANDI! '
                    'Robot ikinci koridora bakiyor.'
                )
            return


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = AisleTransition()

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
