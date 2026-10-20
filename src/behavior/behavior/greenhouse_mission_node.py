#!/usr/bin/env python3
"""Run the tested greenhouse navigation stages in sequence."""

import math
import os
import signal
import subprocess
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


class GreenhouseMission(Node):

    def __init__(self):
        super().__init__('greenhouse_mission')

        self.pose = None
        self.imu_received = False
        self.safety_stop = None
        self.last_seen = {}
        self.child = None

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

        # x, y, yaw ve bunlarin toleranslari.
        self.stages = [
            (
                'greenhouse_route',
                ['--ros-args', '-p', 'end_x:=6.8'],
                1100,
                (7.30, 0.0, 0.0,
                 0.35, 0.25, math.radians(15))
            ),
            (
                'turn_90',
                [],
                240,
                (7.30, 0.0, math.pi / 2,
                 0.35, 0.25, math.radians(15))
            ),
            (
                'aisle_transition',
                [],
                500,
                (7.30, -1.30, math.pi,
                 0.35, 0.30, math.radians(20))
            ),
            (
                'second_aisle',
                [],
                1100,
                (-0.20, -1.30, math.pi,
                 0.20, 0.25, math.radians(25))
            ),
            (
                'return_home',
                [],
                550,
                (0.0, 0.0, 0.0,
                 0.10, 0.10, math.radians(12))
            )
        ]

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (
            p.x, p.y,
            yaw(msg.pose.pose.orientation)
        )
        self.last_seen['odom'] = time.monotonic()

    def on_imu(self, msg):
        self.imu_received = True
        self.last_seen['imu'] = time.monotonic()

    def on_safety(self, msg):
        self.safety_stop = msg.data
        self.last_seen['safety'] = time.monotonic()

    def stop_robot(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def spin_briefly(self, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.1)

    def check_sensors(self):
        if (
            self.pose is None
            or not self.imu_received
            or self.safety_stop is None
        ):
            raise RuntimeError('Sensor verileri eksik.')

        now = time.monotonic()

        for name in ('odom', 'imu', 'safety'):
            if now - self.last_seen.get(name, 0) > 5.0:
                raise RuntimeError(
                    f'{name} verisi guncel degil.'
                )

        if self.safety_stop:
            raise RuntimeError(
                'Guvenlik STOP aktif. Gorev iptal.'
            )

    def check_pose(self, name, target):
        x, y, angle = self.pose
        tx, ty, ta, xtol, ytol, atol = target

        if (
            abs(x - tx) > xtol
            or abs(y - ty) > ytol
            or abs(wrap(angle - ta)) > atol
        ):
            raise RuntimeError(
                f'{name} konum kontrolu BASARISIZ! '
                f'Mevcut: ({x:.2f}, {y:.2f}, '
                f'{math.degrees(angle):.1f} derece), '
                f'beklenen: ({tx:.2f}, {ty:.2f}, '
                f'{math.degrees(ta):.1f} derece)'
            )

        self.get_logger().info(
            f'{name} konum dogrulamasi BASARILI: '
            f'({x:.2f}, {y:.2f}, '
            f'{math.degrees(angle):.1f} derece)'
        )

    def wait_ready(self):
        deadline = time.monotonic() + 30.0

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            if (
                self.pose is not None
                and self.imu_received
                and self.safety_stop is not None
            ):
                self.check_sensors()
                return

        raise RuntimeError(
            'ROS sensorleri 30 saniyede hazir olmadi.'
        )

    def terminate_child(self):
        if self.child is None:
            return

        if self.child.poll() is None:
            try:
                os.killpg(self.child.pid, signal.SIGINT)
                self.child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGTERM)
                try:
                    self.child.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(self.child.pid, signal.SIGKILL)
                    self.child.wait()
            except ProcessLookupError:
                pass

        self.child = None

    def run_stage(self, index, name, args, timeout, target):
        self.check_sensors()

        # Gorev yoneticisinin kendi yayincisi haricinde
        # baska hareket dugumu aktif olmamali.
        if self.count_publishers('/cmd_vel_raw') > 1:
            raise RuntimeError(
                'Baska bir hareket dugumu calisiyor.'
            )

        self.get_logger().info(
            f'ASAMA {index}/5 BASLIYOR: {name}'
        )

        command = ['ros2', 'run', 'behavior', name] + args

        self.child = subprocess.Popen(
            command,
            start_new_session=True
        )

        deadline = time.monotonic() + timeout

        while True:
            rclpy.spin_once(self, timeout_sec=0.1)

            self.check_sensors()

            result = self.child.poll()
            if result is not None:
                if result != 0:
                    raise RuntimeError(
                        f'{name} hata koduyla kapandi: {result}'
                    )
                break

            if time.monotonic() > deadline:
                raise RuntimeError(
                    f'{name} zaman asimina ugradi.'
                )

        self.child = None
        self.stop_robot()
        self.spin_briefly(0.7)

        self.check_sensors()
        self.check_pose(name, target)

        self.get_logger().info(
            f'ASAMA {index}/5 TAMAMLANDI.'
        )

    def run_mission(self):
        self.get_logger().info(
            'Otonom sera gorevi baslatiliyor.'
        )

        self.wait_ready()

        # Ilk Gazebo spawn konumu:
        # X=-0.5, Y=0.65, yaw=0.
        self.check_pose(
            'BASLANGIC',
            (0.0, 0.0, 0.0,
             0.15, 0.15, math.radians(10))
        )

        if self.count_publishers('/cmd_vel_raw') > 1:
            raise RuntimeError(
                'Baska bir navigasyon dugumu aktif.'
            )

        for index, stage in enumerate(self.stages, start=1):
            self.run_stage(index, *stage)

        self.get_logger().info(
            'TUM NAVIGASYON GOREVI BASARIYLA TAMAMLANDI!'
        )
        self.get_logger().info(
            'Robot baslangic noktasinda park halinde.'
        )


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = GreenhouseMission()
    exit_code = 0

    try:
        node.run_mission()
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(
            f'GOREV DURDURULDU: {exc}'
        )
        exit_code = 1
    finally:
        node.terminate_child()
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()

    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
