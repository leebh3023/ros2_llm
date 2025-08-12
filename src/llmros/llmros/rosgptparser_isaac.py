#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import math

class IsaacTwistPublisher(Node):
    def __init__(self):
        super().__init__('isaac_twist_publisher')
        self.subscription = self.create_subscription(String, '/voice_cmd', self.voice_cmd_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('IsaacTwistPublisher node started. Listening to /voice_cmd')

    def voice_cmd_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
            twist = Twist()

            if cmd['action'] == 'move':
                twist.linear.x = float(cmd['params'].get('linear_speed', 0.2))
                twist.angular.z = 0.0

            elif cmd['action'] == 'rotate':
                angular_velocity = float(cmd['params'].get('angular_velocity', 90.0))
                is_clockwise = bool(cmd['params'].get('is_clockwise', True))
                twist.linear.x = 0.0
                twist.angular.z = -math.radians(angular_velocity) if is_clockwise else math.radians(angular_velocity)

            else:
                self.get_logger().warn(f"Unsupported action: {cmd['action']}")
                return

            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(f"Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

        except Exception as e:
            self.get_logger().error(f"[voice_cmd_callback error] {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IsaacTwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
