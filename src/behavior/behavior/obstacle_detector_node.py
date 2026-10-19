"""Detect obstacles in the lidar scan and show them in RViz.

Subscribes:  /scan (sensor_msgs/LaserScan)
Publishes:   /obstacle_detector/state (std_msgs/String): CLEAR, WARN or STOP for the corridor ahead
             /obstacle_detector/front_distance (std_msgs/Float32): nearest point in the corridor (m), inf if none
             /obstacle_detector/markers (visualization_msgs/MarkerArray), in the scan frame:
                 corridor   the strip ahead, green / orange / red by state
                 obstacle   each cluster as a see-through wall from the ground up to the scan plane,
                            red if it is in the corridor, cyan otherwise
                 nearest    a sphere and a distance label on each obstacle's closest point
"""

import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from behavior.obstacle_detection import CLEAR, detect, DetectorParams, STOP, WARN

STATE_COLORS = {
    CLEAR: ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.25),
    WARN: ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.35),
    STOP: ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.45),
}
IN_CORRIDOR = ColorRGBA(r=0.95, g=0.15, b=0.15, a=0.6)
OUTSIDE = ColorRGBA(r=0.1, g=0.8, b=0.9, a=0.35)
LABEL = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
OBSTACLE_NAMESPACES = ('obstacle', 'nearest', 'distance')


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        defaults = DetectorParams()
        self.params = DetectorParams(**{
            name: type(value)(self.declare_parameter(name, value).value)
            for name, value in vars(defaults).items()
        })
        # Height of the scan plane above the ground: obstacle walls are drawn down to the floor
        self.lidar_height = self.declare_parameter('lidar_height', 0.225).value
        self.state_pub = self.create_publisher(String, 'obstacle_detector/state', 10)
        self.distance_pub = self.create_publisher(Float32, 'obstacle_detector/front_distance', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacle_detector/markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.state = None
        self.shown_obstacles = 0

    def on_scan(self, msg):
        result = detect(msg.ranges, msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, self.params)

        self.state_pub.publish(String(data=result.state))
        self.distance_pub.publish(Float32(data=result.front_distance))
        if result.state != self.state:
            distance = '' if math.isinf(result.front_distance) else f' ({result.front_distance:.2f} m)'
            # One call site per severity: rclpy refuses to log from the same line with a different one
            if result.state == STOP:
                self.get_logger().warning(f'{result.state}{distance}')
            else:
                self.get_logger().info(f'{result.state}{distance}')
            self.state = result.state

        if self.marker_pub.get_subscription_count() > 0:
            self.marker_pub.publish(self.make_markers(msg.header, result))

    def make_markers(self, header, result):
        # Markers are replaced in place by id; only the ids of obstacles that went away are deleted
        # (clearing everything with DELETEALL on every scan makes RViz flicker)
        markers = MarkerArray()
        for i in range(len(result.obstacles), self.shown_obstacles):
            for ns in OBSTACLE_NAMESPACES:
                markers.markers.append(Marker(header=header, ns=ns, id=i, action=Marker.DELETE))
        self.shown_obstacles = len(result.obstacles)

        def marker(ns, marker_id, marker_type, color):
            m = Marker(header=header, ns=ns, id=marker_id, type=marker_type, color=color)
            m.pose.orientation.w = 1.0
            markers.markers.append(m)
            return m

        corridor = marker('corridor', 0, Marker.CUBE, STATE_COLORS[result.state])
        length = self.params.warn_distance
        corridor.pose.position.x = length / 2
        corridor.pose.position.z = -self.lidar_height + 0.01
        corridor.scale.x = length
        corridor.scale.y = 2 * self.params.corridor_half_width
        corridor.scale.z = 0.02

        bottom, top = -self.lidar_height, 0.0
        for i, obstacle in enumerate(result.obstacles):
            color = IN_CORRIDOR if obstacle.in_corridor else OUTSIDE
            wall = marker('obstacle', i, Marker.TRIANGLE_LIST, color)
            wall.scale.x = wall.scale.y = wall.scale.z = 1.0
            # Two triangles per pair of neighbouring scan points
            for (x0, y0), (x1, y1) in zip(obstacle.points[:-1], obstacle.points[1:]):
                a, b = Point(x=float(x0), y=float(y0), z=bottom), Point(x=float(x1), y=float(y1), z=bottom)
                c, d = Point(x=float(x1), y=float(y1), z=top), Point(x=float(x0), y=float(y0), z=top)
                wall.points += [a, b, c, a, c, d]

            x, y = (float(v) for v in obstacle.nearest)
            nearest = marker('nearest', i, Marker.SPHERE, color)
            nearest.pose.position.x, nearest.pose.position.y = x, y
            nearest.scale.x = nearest.scale.y = nearest.scale.z = 0.08

            label = marker('distance', i, Marker.TEXT_VIEW_FACING, LABEL)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.15
            label.scale.z = 0.1
            label.text = f'{obstacle.distance:.2f} m'
        return markers


def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
