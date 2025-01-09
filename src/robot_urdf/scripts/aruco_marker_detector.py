#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

class PrintNode(Node):
    def __init__(self):
        super().__init__('print_node')
        self.get_logger().info('PrintNode has been started.')
        self.timer = self.create_timer(1.0, self.timer_callback)  # Create a timer that fires every second

    def timer_callback(self):
        self.get_logger().info('Hello, this is a message from PrintNode!')

def main(args=None):
    rclpy.init(args=args)
    node = PrintNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()