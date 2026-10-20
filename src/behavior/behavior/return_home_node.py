#!/usr/bin/env python3
"""Return from the second aisle to the original starting pose."""

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


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class ReturnHome(Node):

    def __init__(self):
        super().__init__('return_home_navigation')

        self.pose = None
        self.imu = None
        self.safety = None
        self.timestamps = {}

        self.initial_imu = None
        self.initial_odom_yaw = None

        self.targets = []
        self.index = 0
        self.mode = 'WAIT'
        self.done = False

        self.phase_start = time.monotonic()
        self.hold_until = 0.0
        self.last_log = 0.0

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
            'Baslangica donus ve park sistemi hazir.'
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

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def finish(self, message):
        self.command()
        self.done = True
        self.get_logger().info(message)

    def heading(self):
        delta = wrap(self.imu - self.initial_imu)
        return wrap(self.initial_odom_yaw + delta)

    def angular_speed(self, error):
        if abs(error) <= math.radians(3):
            return 0.0

        speed = min(
            0.35,
            max(0.12, 1.2 * abs(error))
        )
        return math.copysign(speed, error)

    def change_mode(self, mode):
        self.command()
        self.mode = mode
        self.phase_start = time.monotonic()

    def control(self):
        if self.done:
            return

        now = time.monotonic()

        if (
            self.pose is None
            or self.imu is None
            or self.safety is None
        ):
            self.command()

            if now - self.phase_start > 30.0:
                self.finish('Sensorler gelmedi. Iptal.')
            return

        if any(
            now - self.timestamps.get(k, 0) > 5.0
            for k in ('odom', 'imu', 'safety')
        ):
            self.finish('Sensor verisi eskidi. STOP.')
            return

        if self.safety:
            self.finish('Guvenlik STOP! Gorev iptal.')
            return

        if self.mode == 'WAIT':
            x, y, angle = self.pose

            # Test edilmis ikinci koridor bitisi.
            if not (
                -0.40 <= x <= 0.05
                and -1.50 <= y <= -1.00
                and abs(wrap(angle - math.pi))
                    <= math.radians(25)
            ):
                self.finish(
                    'Baslangic konumu uygun degil. '
                    'Robot hareket ettirilmedi.'
                )
                return

            self.initial_imu = self.imu
            self.initial_odom_yaw = angle

            # Odom (0,0), Gazebo (-0.5,+0.65).
            # Once batida guvenli mesafeye cik.
            self.targets = [
                (-0.55, y, 'BATI ACIK ALAN'),
                (-0.55, 0.0, 'KUZEY GECISI'),
                (0.0, 0.0, 'BASLANGIC')
            ]

            self.index = 0
            self.change_mode('DRIVE')

            self.get_logger().info(
                f'BASLANGIC: ({x:.3f}, {y:.3f})'
            )
            self.get_logger().info(
                'ROTA: Bati acik alan -> '
                'Kuzey gecisi -> Baslangic -> Park'
            )
            return

        if now - self.phase_start > 240.0:
            self.finish(
                'Asama zaman asimi. Robot durduruldu.'
            )
            return

        if self.mode == 'HOLD':
            self.command()

            if now >= self.hold_until:
                self.change_mode('DRIVE')
            return

        if self.mode == 'PARK':
            # Ilk Gazebo baslangic yonu: 0 derece.
            error = wrap(0.0 - self.heading())

            if abs(error) <= math.radians(3):
                x, y, _ = self.pose
                self.finish(
                    'TAM SERA TURU TAMAMLANDI! '
                    f'Park konumu: ({x:.3f}, {y:.3f}), '
                    f'yon hatasi: '
                    f'{math.degrees(error):.1f} derece'
                )
                return

            self.command(
                angular=self.angular_speed(error)
            )

            if now - self.last_log > 2.0:
                self.get_logger().info(
                    f'PARK | Kalan aci: '
                    f'{math.degrees(error):.1f} derece'
                )
                self.last_log = now
            return

        # Bir sonraki koordinata ilerle.
        tx, ty, name = self.targets[self.index]
        x, y, _ = self.pose

        dx = tx - x
        dy = ty - y
        remaining = math.hypot(dx, dy)

        if remaining <= 0.05:
            self.command()
            self.get_logger().info(
                f'{name} TAMAM | '
                f'Konum hatasi: {remaining:.3f} m'
            )

            self.index += 1

            if self.index == len(self.targets):
                self.change_mode('PARK')
                self.get_logger().info(
                    'Baslangic noktasina gelindi. '
                    'Park yonu ayarlaniyor.'
                )
            else:
                self.change_mode('HOLD')
                self.hold_until = now + 1.0
            return

        target_heading = math.atan2(dy, dx)
        error = wrap(
            target_heading - self.heading()
        )

        # Buyuk yon hatasinda once yerinde don.
        if abs(error) > math.radians(8):
            linear = 0.0
        elif remaining < 0.15:
            linear = 0.05
        else:
            linear = 0.08

        self.command(
            linear,
            self.angular_speed(error)
        )

        if now - self.last_log > 2.0:
            self.get_logger().info(
                f'{name} | '
                f'Kalan: {remaining:.2f} m | '
                f'Yon hatasi: '
                f'{math.degrees(error):.1f} derece'
            )
            self.last_log = now


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = ReturnHome()

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
