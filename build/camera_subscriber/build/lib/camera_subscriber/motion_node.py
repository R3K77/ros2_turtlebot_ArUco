#!/usr/bin/env python3
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from geometry_msgs.msg import Twist  # Message type for velocity commands


class CircleMotionPublisher(Node):
    def __init__(self):
        super().__init__('circle_motion_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.twist_msg = Twist()
        self.linear_speed = 0.2  
        self.angular_speed = 0.5  
        self.get_logger().info('CircleMotionPublisher node has been started.')

    def timer_callback(self):
        self.twist_msg.linear.x = self.linear_speed
        self.twist_msg.angular.z = self.angular_speed
        self.publisher_.publish(self.twist_msg)
        self.get_logger().info(f'Publishing Twist: linear={self.linear_speed}, angular={self.angular_speed}')


def main(args=None):
    rclpy.init(args=args)
    node = CircleMotionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.twist_msg.linear.x = 0.0
        node.twist_msg.angular.z = 0.0
        node.publisher_.publish(node.twist_msg)
        node.get_logger().info('Stopping the robot and shutting down node.')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
