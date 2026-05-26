#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathPublisher(Node):

    def __init__(self):
        super().__init__('odom_path_publisher')

        # Parameter for odometry topic
        self.declare_parameter(
            'odom_topic',
            '/uav1/estimation_manager/odom_main'
        )

        odom_topic = self.get_parameter(
            'odom_topic'
        ).get_parameter_value().string_value

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Publisher for path
        self.path_publisher = self.create_publisher(
            Path,
            '/uav1/path',
            10
        )

        # Path message
        self.path = Path()
        self.path.header.frame_id = 'map'

        self.get_logger().info(
            f'Subscribed to: {odom_topic}'
        )

    def odom_callback(self, msg):

        # Create PoseStamped from Odometry
        pose = PoseStamped()

        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append pose to path
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)

        # Publish path
        self.path_publisher.publish(self.path)

        # Print current position
        self.get_logger().info(
            f'Position -> '
            f'x: {pose.pose.position.x:.2f}, '
            f'y: {pose.pose.position.y:.2f}, '
            f'z: {pose.pose.position.z:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = OdomPathPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
