"""Detect red/green apples and RED/GREEN wall checkpoints in the camera image.

Subscribes:  /camera/image_raw (sensor_msgs/Image)
Publishes:   /color_detector/<label> (geometry_msgs/PointStamped), one topic per label, only while it is seen:
                 red_apple, green_apple, red_checkpoint, green_checkpoint
                 point.x = horizontal offset of the blob from the image center, -1 (left) .. +1 (right)
                 point.y = vertical offset, -1 (top) .. +1 (bottom)
                 point.z = fraction of the image covered by the blob (grows as the robot gets closer)
             /color_detector/debug_image (sensor_msgs/Image): detections drawn on the camera image
"""

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from behavior.color_detection import detect, draw_debug, HsvThresholds, LABELS


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')
        defaults = HsvThresholds()
        self.thresholds = HsvThresholds(**{
            name: self.declare_parameter(name, value).value
            for name, value in vars(defaults).items()
        })
        self.min_area = float(self.declare_parameter('min_area', 150.0).value)
        self.bridge = CvBridge()
        self.pubs = {
            label: self.create_publisher(PointStamped, f'color_detector/{label}', 10)
            for label in LABELS
        }
        self.debug_pub = self.create_publisher(Image, 'color_detector/debug_image', 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.seen = set()

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detections, masks = detect(bgr, self.thresholds, self.min_area)

        for label, det in detections.items():
            out = PointStamped()
            out.header = msg.header
            out.point.x = det.offset_x
            out.point.y = det.offset_y
            out.point.z = det.area_ratio
            self.pubs[label].publish(out)
        for label in sorted(set(detections) ^ self.seen):
            self.get_logger().info(f'{label} {"in view" if label in detections else "out of view"}')
        self.seen = set(detections)

        if self.debug_pub.get_subscription_count() > 0:
            debug = self.bridge.cv2_to_imgmsg(draw_debug(bgr, detections, masks), 'bgr8')
            debug.header = msg.header
            self.debug_pub.publish(debug)


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
