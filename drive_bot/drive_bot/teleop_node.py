#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class TeleopNode(Node):
    """Node for teleoperating the robot using keyboard input."""

    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_bindings = {
            'w': (1.0, 0.0),  # Forward
            's': (-1.0, 0.0),  # Backward
            'a': (0.0, 1.0),   # Left
            'd': (0.0, -1.0),  # Right
        }
        self.speed = 0.5  # Linear speed (m/s)
        self.turn = 1.0   # Angular speed (rad/s)
        self.get_logger().info('Teleop node started. Use WASD to move, q to quit.')

    def run(self):
        """Main loop to process keypresses and publish velocity commands."""
        while rclpy.ok():
            try:
                key = input("Enter command (w/a/s/d/q): ").strip().lower()
                twist = Twist()
                if key in self.move_bindings:
                    x, th = self.move_bindings[key]
                    twist.linear.x = x * self.speed
                    twist.angular.z = th * self.turn
                elif key == 'q':
                    break
                self.publisher_.publish(twist)
            except (KeyboardInterrupt, EOFError):
                break
            except Exception as e:
                self.get_logger().error(f"Error in teleop: {str(e)}")
                break
        twist = Twist()
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Teleop node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()