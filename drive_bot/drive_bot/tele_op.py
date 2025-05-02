#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vel = Twist()
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.get_logger().info('Teleop node started. Use WASD to move, Q to quit.')
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1).lower()
            else:
                key = ''
        except Exception as e:
            self.get_logger().error(f'Key read error: {e}')
            key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        key = self.get_key()
        self.get_logger().info(f'Key pressed: {key}')
        if key == 'w':
            self.vel.linear.x = self.linear_speed
            self.vel.angular.z = 0.0
        elif key == 's':
            self.vel.linear.x = -self.linear_speed
            self.vel.angular.z = 0.0
        elif key == 'a':
            self.vel.linear.x = 0.0
            self.vel.angular.z = self.angular_speed
        elif key == 'd':
            self.vel.linear.x = 0.0
            self.vel.angular.z = -self.angular_speed
        elif key == 'q':
            self.get_logger().info('Teleop node shutting down.')
            rclpy.shutdown()
            return
        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0

        self.get_logger().info(f'Publishing Twist to /diff_cont/cmd_vel: linear.x={self.vel.linear.x}, angular.z={self.vel.angular.z}')
        self.publisher_.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down teleop node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()