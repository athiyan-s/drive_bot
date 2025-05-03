#!/usr/bin/env python3
"""ROS 2 node for navigating a differential drive robot using Bug0 and Bug2 algorithms."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class BugNavigationNode(Node):
    """Node for navigating a robot using Bug0 and Bug2 algorithms in a Gazebo environment."""

    def __init__(self):
        """Initialize the navigation node with publishers, subscribers, and parameters."""
        super().__init__('bug_navigation_node')
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        # Parameters
        self.current_pose = None
        self.laser_ranges = []
        self.state = 'STOP'
        self.algorithm = 'BUG0'  # Switch to 'BUG2' to use Bug2 algorithm
        self.min_distance = 0.5  # Minimum distance to obstacle (m)
        self.linear_speed = 0.3  # Linear speed (m/s)
        self.angular_speed = 0.5  # Angular speed (rad/s)
        self.heading_threshold = 0.1  # Heading error threshold for turn-in-place (radians)
        self.hit_point = None
        self.m_line = None  # For Bug2: stores the M-line (start to goal)
        self.is_shutdown = False

        # Hardcode the goal position (x, y, orientation in radians)
        self.goal = Pose()
        self.goal.position.x = 2.0  # Example goal at x=2.0
        self.goal.position.y = -2.0  # Example goal at y=-2.0
        self.goal.position.z = 0.0
        # Set orientation (0 radians, facing forward along x-axis)
        quat = quaternion_from_euler(0.0, 0.0, 0.0)
        self.goal.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.get_logger().info(f'Initialized {self.algorithm} navigation node with goal at x={self.goal.position.x}, y={self.goal.position.y}')

    def scan_callback(self, msg):
        """Store laser scan data and log for debugging."""
        if self.is_shutdown:
            return
        self.laser_ranges = msg.ranges
        # Log a sample of the ranges to confirm data is received
        front_indices = list(range(0, 60)) + list(range(660, 720))  # Adjusted for 720 samples
        sample_ranges = [self.laser_ranges[i] for i in front_indices if not math.isinf(self.laser_ranges[i])]
        self.get_logger().info(f'Received laser ranges (front arc, {len(sample_ranges)} valid): {sample_ranges}')
        # Log minimum range in front arc
        if sample_ranges:
            min_range = min(sample_ranges)
            self.get_logger().info(f'Minimum range in front arc: {min_range} m')
        else:
            self.get_logger().info('No finite ranges in front arc')

    def odom_callback(self, msg):
        """Update current pose and trigger navigation."""
        if self.is_shutdown:
            return
        self.current_pose = msg.pose.pose
        # Set the M-line for Bug2 once we have the initial pose
        if self.m_line is None and self.algorithm == 'BUG2':
            self.m_line = (self.current_pose.position, self.goal.position)
            self.state = 'GO_TO_GOAL'
        elif self.state == 'STOP':
            self.state = 'GO_TO_GOAL'
        self.navigate()

    def get_distance_to_goal(self):
        """Calculate Euclidean distance to the goal."""
        if self.current_pose is None or self.goal is None:
            return float('inf')
        dx = self.goal.position.x - self.current_pose.position.x
        dy = self.goal.position.y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def get_heading_error(self):
        """Calculate the angular error to the goal."""
        if self.current_pose is None or self.goal is None:
            return 0.0
        dx = self.goal.position.x - self.current_pose.position.x
        dy = self.goal.position.y - self.current_pose.position.y
        goal_angle = math.atan2(dy, dx)
        orientation = self.current_pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        error = goal_angle - yaw
        # Normalize angle to [-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        return error

    def obstacle_detected(self):
        """Check if an obstacle is within the minimum distance in the front arc."""
        if not self.laser_ranges:
            self.get_logger().warning('No laser ranges received!')
            return False
        # Consider front 60 degrees (30 degrees left and right)
        front_indices = list(range(0, 60)) + list(range(660, 720))  # Adjusted for 720 samples
        valid_ranges = [self.laser_ranges[i] for i in front_indices if not math.isinf(self.laser_ranges[i])]
        self.get_logger().info(f'Checking obstacle: front laser ranges = {valid_ranges}')
        return any(r < self.min_distance for r in valid_ranges) if valid_ranges else False

    def navigate(self):
        """Execute the navigation state machine."""
        if self.state == 'STOP' or self.current_pose is None or self.goal is None or self.is_shutdown:
            self.stop_robot()
            return

        twist = Twist()
        # Check if goal is reached
        if self.get_distance_to_goal() < 0.1:
            self.state = 'STOP'
            self.get_logger().info('Goal reached!')
            self.stop_robot()
            return

        try:
            if self.algorithm == 'BUG0':
                self.bug0(twist)
            elif self.algorithm == 'BUG2':
                self.bug2(twist)
            if not self.is_shutdown:
                self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Navigation error: {str(e)}')
            self.stop_robot()

    def stop_robot(self):
        """Publish a zero velocity command to stop the robot."""
        if not self.is_shutdown:
            self.cmd_vel_pub.publish(Twist())

    def bug0(self, twist):
        """Implement Bug0 algorithm: go to goal unless obstacle, then follow wall."""
        if self.state == 'GO_TO_GOAL':
            if not self.obstacle_detected():
                # Move towards goal with turn-in-place if needed
                angle_error = self.get_heading_error()
                if abs(angle_error) > self.heading_threshold:
                    # Turn in place to face the goal
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed * (1.0 if angle_error > 0 else -1.0)
                else:
                    # Move straight with small corrections
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed * max(min(angle_error, 1.0), -1.0)
            else:
                # Obstacle detected, switch to wall following
                self.state = 'FOLLOW_WALL'
                self.hit_point = (self.current_pose.position.x, self.current_pose.position.y)
                self.get_logger().info('Obstacle detected, switching to FOLLOW_WALL')
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
        elif self.state == 'FOLLOW_WALL':
            if not self.obstacle_detected() and self.get_distance_to_goal() < self.get_distance_to_hit_point():
                # Clear path to goal and closer than hit point, resume going to goal
                self.state = 'GO_TO_GOAL'
                self.get_logger().info('Clear path, switching to GO_TO_GOAL')
            else:
                # Follow wall by maintaining distance
                twist.angular.z = self.angular_speed
                front_indices = list(range(0, 60)) + list(range(660, 720))
                valid_ranges = [self.laser_ranges[i] for i in front_indices if not math.isinf(self.laser_ranges[i])]
                if valid_ranges:
                    avg_distance = sum(valid_ranges) / len(valid_ranges)
                    twist.linear.x = min(self.linear_speed, max(0.0, avg_distance - self.min_distance))

    def bug2(self, twist):
        """Implement Bug2 algorithm: follow M-line, circle obstacle, leave at M-line closer to goal."""
        if self.state == 'GO_TO_GOAL':
            if not self.obstacle_detected():
                # Move towards goal along M-line with turn-in-place if needed
                angle_error = self.get_heading_error()
                if abs(angle_error) > self.heading_threshold:
                    # Turn in place to face the goal
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed * (1.0 if angle_error > 0 else -1.0)
                else:
                    # Move straight with small corrections
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed * max(min(angle_error, 1.0), -1.0)
            else:
                # Obstacle detected, start wall following
                self.state = 'FOLLOW_WALL'
                self.hit_point = (self.current_pose.position.x, self.current_pose.position.y)
                self.get_logger().info('Obstacle detected, switching to FOLLOW_WALL')
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
        elif self.state == 'FOLLOW_WALL':
            # Check if we intersect the M-line and are closer to the goal
            if self.is_on_m_line() and self.get_distance_to_goal() < self.get_distance_to_hit_point():
                self.state = 'GO_TO_GOAL'
                self.get_logger().info('Intersected M-line closer to goal, switching to GO_TO_GOAL')
            else:
                # Follow wall by maintaining distance
                twist.angular.z = self.angular_speed
                front_indices = list(range(0, 60)) + list(range(660, 720))
                valid_ranges = [self.laser_ranges[i] for i in front_indices if not math.isinf(self.laser_ranges[i])]
                if valid_ranges:
                    avg_distance = sum(valid_ranges) / len(valid_ranges)
                    twist.linear.x = min(self.linear_speed, max(0.0, avg_distance - self.min_distance))

    def get_distance_to_hit_point(self):
        """Calculate distance to the hit point."""
        if self.current_pose is None or self.hit_point is None:
            return float('inf')
        dx = self.current_pose.position.x - self.hit_point[0]
        dy = self.current_pose.position.y - self.hit_point[1]
        return math.sqrt(dx**2 + dy**2)

    def is_on_m_line(self):
        """Check if the current position is close to the M-line (within a threshold)."""
        if self.current_pose is None or self.m_line is None:
            return False
        # M-line is defined by two points: start (m_line[0]) and goal (m_line[1])
        start, goal = self.m_line
        px, py = self.current_pose.position.x, self.current_pose.position.y
        # Line equation: (y - y1)(x2 - x1) - (x - x1)(y2 - y1) = 0
        # Distance from point to line: | (y - y1)(x2 - x1) - (x - x1)(y2 - y1) | / sqrt((x2 - x1)^2 + (y2 - y1)^2)
        dx = goal.x - start.x
        dy = goal.y - start.y
        numerator = abs((py - start.y) * dx - (px - start.x) * dy)
        denominator = math.sqrt(dx**2 + dy**2)
        if denominator == 0:
            return False
        distance = numerator / denominator
        return distance < 0.1  # Threshold for being "on" the M-line

    def shutdown(self):
        """Clean up resources on shutdown."""
        self.is_shutdown = True
        self.stop_robot()
        self.destroy_node()


def main(args=None):
    """Initialize and run the BugNavigationNode."""
    rclpy.init(args=args)
    node = BugNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation node interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()