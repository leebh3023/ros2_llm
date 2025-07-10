#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import json
import copy
import math
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from rclpy.executors import SingleThreadedExecutor

class TurtlesimController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')
        self.create_subscription(String, '/voice_cmd', self.voice_cmd_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose = Pose()
        self.thread_executor = ThreadPoolExecutor(max_workers=1)
        self.move_executor = SingleThreadedExecutor()
        threading.Thread(target=self.move_executor.spin, daemon=True).start()
        print('Turtlesim Controller Started.')

    def pose_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.pose = msg

    def voice_cmd_callback(self, msg):
        try:
            cmd = json.loads(msg.data)

            if cmd['action'] == 'move':
                linear_speed = cmd['params'].get('linear_speed', 0.2)
                distance = cmd['params'].get('distance', 1.0)
                is_forward = cmd['params'].get('is_forward', True)
                self.thread_executor.submit(self.move, linear_speed, distance, is_forward)

            elif cmd['action'] == 'rotate':
                angular_velocity = cmd['params'].get('angular_velocity', 1.0)
                angle = cmd['params'].get('angle', 90.0)
                is_clockwise = cmd['params'].get('is_clockwise', True)
                self.thread_executor.submit(self.rotate, angular_velocity, angle, is_clockwise)

        except Exception as e:
            print(f'[voice_cmd_callback error] {e}')

    def get_distance(self, start, end):
        return math.sqrt((end.x - start.x)**2 + (end.y - start.y)**2)

    def move(self, speed, distance, forward):
        direction = 1 if forward else -1
        twist = Twist()
        twist.linear.x = abs(speed) * direction
        start = copy.deepcopy(self.pose)

        while self.get_distance(start, self.pose) < distance:
            self.velocity_publisher.publish(twist)
            self.move_executor.spin_once(timeout_sec=0.1)

        twist.linear.x = 0.0
        self.velocity_publisher.publish(twist)
        print('Move complete.')

    def rotate(self, angular_speed, angle, clockwise):
        twist = Twist()
        angular_speed_rad = math.radians(angular_speed)
        twist.angular.z = -abs(angular_speed_rad) if clockwise else abs(angular_speed_rad)
        start_theta = self.theta
        rotated = 0.0

        while rotated < angle:
            self.velocity_publisher.publish(twist)
            rotated = math.degrees(abs(start_theta - self.theta))
            time.sleep(0.01)

        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        print('Rotate complete.')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
