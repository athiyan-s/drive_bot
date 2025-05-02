#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class BugNavigationNode(Node):
    """Node for navigating the robot using Bug0 and Bug1 algorithms."""

    def __init__(self):
        super().__init__('bug_navigation_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.goal = None
        self.current_pose = None
        self.laser_ranges = []
        self.state = 'STOP'
        self.algorithm = 'BUG0'  # Can be 'BUG0' or 'BUG1'
        self.min_distance = 0.5  # Minimum distance to obstacle
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.hit_point = None
        self.leave_point = None
        self.get_logger().info(f'Started {self.algorithm} navigation node.')

    def goal_callback(self, msg):
        """Callback for goal pose from RViz."""
        self.goal = msg.pose
        self.state = 'GO_TO_GOAL'
        self.get_logger().info('Received new goal pose.')

    def scan_callback(self, msg):
        """Callback for laser scan data."""
        self.laser_ranges = msg.ranges

    def odom_callback(self, msg):
        """Callback for odometry data."""
        self.current_pose = msg.pose.pose
        self.navigate()

    def get_distance_to_goal(self):
        """Calculate Euclidean distance to the goal."""
        if self.current_pose is None or self.goal is None:
            return float('inf')
        dx = self.goal.position.x - self.current_pose.position.x
        dy = self.goal.position.y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def get_heading_error(self):
        """Calculate the heading error to the goal."""
        if self.current_pose is None or self.goal is None:
            return 0.0
        dx = self.goal.position.x - self.current_pose.position.x
        dy = self.goal.position.y - self.current_pose.position.y
        goal_angle = math.atan2(dy, dx)
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return goal_angle - yaw

    def obstacle_detected(self):
        """Check if an obstacle is within the minimum distance."""
        if not self.laser_ranges:
            return False
        front_ranges = self.laser_ranges[0:30] + self.laser_ranges[-30:]
        valid_ranges = [r for r in front_ranges if not math.isinf(r)]
        return any(r < self.min_distance for r in valid_ranges)

    def navigate(self):
        """Main navigation logic for Bug0 and Bug1 algorithms."""
        if self.state == 'STOP' or self.current_pose is None or self.goal is None:
            return

        twist = Twist()
        if self.get_distance_to_goal() < 0.1:
            self.state = 'STOP'
            self.get_logger().info('Goal reached!')
            self.cmd_vel_pub.publish(twist)
            return

        try:
            if self.algorithm == 'BUG0':
                self.bug0(twist)
            elif self.algorithm == 'BUG1':
                self.bug1(twist)
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Navigation error: {str(e)}")
            self.cmd_vel_pub.publish(Twist())

    def bug0(self, twist):
        """Implementation of Bug0 algorithm."""
        if self.state == 'GO_TO_GOAL':
            if not self.obstacle_detected():
                angle_error = self.get_heading_error()
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed * max(min(angle_error, 1.0), -1.0)
            else:
                self.state = 'FOLLOW_WALL'
                self.hit_point = (self.current_pose.position.x, self.current_pose.position.y)
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
        elif self.state == 'FOLLOW_WALL':
            if not self.obstacle_detected():
                self.state = 'GO_TO_GOAL'
            else:
                twist.angular.z = self.angular_speed
                front_ranges = self.laser_ranges[0:30] + self.laser_ranges[-30:]
                valid_ranges = [r for r in front_ranges if not math.isinf(r)]
                if valid_ranges:
                    avg_distance = sum(valid_ranges) / len(valid_ranges)
                    twist.linear.x = min(self.linear_speed, avg_distance - self.min_distance)

    def bug1(self, twist):
        """Implementation of Bug1 algorithm."""
        if self.state == 'GO_TO_GOAL':
            if not self.obstacle_detected():
                angle_error = self.get_heading_error()
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed * max(min(angle_error, 1.0), -1.0)
            else:
                self.state = 'CIRCLE_OBSTACLE'
                self.hit_point = (self.current_pose.position.x, self.current_pose.position.y)
                self.leave_point = None
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
        elif self.state == 'CIRCLE_OBSTACLE':
            if not self.obstacle_detected():
                self.state = 'GO_TO_GOAL'
            else:
                twist.angular.z = self.angular_speed
                front_ranges = self.laser_ranges[0:30] + self.laser_ranges[-30:]
                valid_ranges = [r for r in front_ranges if not math.isinf(r)]
                if valid_ranges:
                    avg_distance = sum(valid_ranges) / len(valid_ranges)
                    twist.linear.x = min(self.linear_speed, avg_distance - self.min_distance)
                dx = self.current_pose.position.x - self.hit_point[0]
                dy = self.current_pose.position.y - self.hit_point[1]
                if math.sqrt(dx**2 + dy**2) < 0.2 and self.leave_point is None:
                    self.leave_point = (self.current_pose.position.x, self.current_pose.position.y)
                    self.state = 'GO_TO_GOAL'

def main(args=None):
    rclpy.init(args=args)
    node = BugNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation node interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        if rclpy.ok():
            twist = Twist()
            node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()