#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class Turn90(Node):

    def __init__(self):
        super().__init__('turn_90')

        self.yaw = None
        self.safety_stop = None
        self.last_imu = 0.0
        self.last_safety = 0.0

        self.target_yaw = None
        self.start_time = None
        self.wait_start = time.monotonic()
        self.last_log = 0.0
        self.done = False

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel_raw', 10
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
            'IMU ile 90 derece donus testi hazir.'
        )

    def on_imu(self, msg):
        self.yaw = get_yaw(msg.orientation)
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

        # Sensör ve güvenlik verileri gelmeden hareket etme.
        if self.yaw is None or self.safety_stop is None:
            self.stop()
            if now - self.wait_start > 20.0:
                self.finish('Sensor bekleme suresi asildi.')
            return

        # Eski verilerle robotu hareket ettirme.
        if (now - self.last_imu > 5.0 or
                now - self.last_safety > 5.0):
            self.finish('Guncel sensor verisi yok. Duruldu.')
            return

        if self.safety_stop:
            self.finish('Guvenlik uyarisi! Donus iptal edildi.')
            return

        # İlk IMU yönüne göre 90 derece sola dön.
        if self.target_yaw is None:
            self.target_yaw = normalize_angle(
                self.yaw + math.pi / 2
            )
            self.start_time = now

            self.get_logger().info(
                f'Baslangic: {math.degrees(self.yaw):.1f} derece | '
                f'Hedef: {math.degrees(self.target_yaw):.1f} derece'
            )

        if now - self.start_time > 120.0:
            self.finish('Donus suresi asildi. Robot durduruldu.')
            return

        error = normalize_angle(
            self.target_yaw - self.yaw
        )

        # 3 derecelik hedef toleransı.
        if abs(error) < math.radians(3):
            self.finish(
                f'90 DERECE DONUS TAMAMLANDI! '
                f'Son yon: {math.degrees(self.yaw):.1f}'
            )
            return

        # Hedefe yaklaşırken yavaşla.
        speed = min(
            0.45,
            max(0.12, 1.2 * abs(error))
        )

        cmd = Twist()
        cmd.angular.z = math.copysign(speed, error)
        self.publisher.publish(cmd)

        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f'Yon: {math.degrees(self.yaw):.1f} | '
                f'Kalan aci: {math.degrees(error):.1f} derece'
            )
            self.last_log = now


def main():
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = Turn90()

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
