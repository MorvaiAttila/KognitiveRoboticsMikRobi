#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TestMarkerPublisher(Node):
    def __init__(self):
        super().__init__('test_marker_pub')
        self.publisher = self.create_publisher(Marker, '/test_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_marker)
        self.id = 0

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'test'
        marker.id = self.id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0  # default rotation

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0  # forever

        self.publisher.publish(marker)
        self.get_logger().info(f'Published test marker id={self.id}')
        self.id += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()