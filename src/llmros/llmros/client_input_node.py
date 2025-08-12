#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ClientInputNode(Node):
    def __init__(self):
        super().__init__('client_input_node')
        self.publisher_ = self.create_publisher(String, 'text_command', 10)
        threading.Thread(target=self.get_input_loop, daemon=True).start()
        self.get_logger().info('ClientInputNode started. Enter text commands:')

    def get_input_loop(self):
        while rclpy.ok():
            text = input(">>> ")
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClientInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
