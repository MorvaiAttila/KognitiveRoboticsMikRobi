#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_pose
from cv_bridge import CvBridge
import cv2
import numpy as np

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class LineMapper(Node):
    def __init__(self):
        super().__init__('line_mapper')

        # TF buffer és listener a /map -> /base_link lekérdezéshez
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publikációk
        self.marker_pub = self.create_publisher(Marker, '/line_breaks', 10)
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # Segédváltozók
        self.bridge = CvBridge()
        self.break_id = 0
        self.break_positions = []

        # Szakadásdetektálás változó, alapértelmezett a false
        self.break_detected = False

        # Kamera képfeldolgozás subscription
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)

    # Path készítése a map frame-re
    def update_path(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            self.path_msg.header.stamp = pose.header.stamp
            self.path_msg.poses.append(pose)
            self.path_pub.publish(self.path_msg)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform for path update: {str(e)}')


##odometria kiegészítése lidar adatokkal
    # Kép callback, itt történik a képfeldolgozás. A végén frissít a path-nek
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        height, width = frame.shape
        roi = frame[int(height * 2/3):, :]  # Alsó 1/3 kivágása

        _, binary = cv2.threshold(roi, 200, 255, cv2.THRESH_BINARY)

        white_pixels = cv2.countNonZero(binary)
        total_pixels = binary.size
        white_ratio = (white_pixels / total_pixels) * 100

        # fehér pixel arány treshold 18%
        if white_ratio < 18.0:
            if not self.break_detected:
                self.get_logger().info(f'Line break detected! (white_ratio={white_ratio:.2f}%)')
                self.break_detected = True
                self.publish_break_marker()
        # hiszterézis        
        if white_ratio > 20.0:
            self.break_detected = False
        
        self.update_path()

    # Marker letolása
    def publish_break_marker(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)

            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y

            for x, y in self.break_positions:
                dist = np.hypot(current_x - x, current_y - y)
                if dist < 0.3:
                    self.get_logger().info(f'Line break already published at (x={x:.2f}, y={y:.2f}), skipping.')
                    return

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'line_breaks'
            marker.id = self.break_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = trans.transform.translation.x
            marker.pose.position.y = trans.transform.translation.y
            marker.pose.position.z = 0.25
            marker.pose.orientation = trans.transform.rotation

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Lila
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0 # infinityyyyyy

            self.marker_pub.publish(marker)
            self.get_logger().info(
                f'Published marker {self.break_id} at (x={current_x:.2f}, y={current_y:.2f})'
            )

            self.break_positions.append((current_x, current_y))
            self.break_id += 1

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform for marker: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LineMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
