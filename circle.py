# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_drawer')

        # Publisher to command the turtle's velocity
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Movement parameters
        self.speed_linear = 1.0  # units per second
        self.radius_circle = 2.0  # radius of the circle
        self.speed_angular = self.speed_linear / self.radius_circle  # angular velocity

        # Time to complete full circle = (2 * PI * R) / linear speed
        self.circle_duration = (2 * 3.1416 * self.radius_circle) / self.speed_linear

        # Time tracking
        self.start_time = self.get_clock().now()

        # Timer to update movement at fixed intervals
        self.timer_interval = 0.1  # seconds
        self.timer = self.create_timer(self.timer_interval, self.send_velocity_command)

    def send_velocity_command(self):
        current_time = self.get_clock().now()
        time_elapsed = (current_time - self.start_time).nanoseconds / 1e9

        if time_elapsed < self.circle_duration:
            # Move in a circular path
            move = Twist()
            move.linear.x = self.speed_linear
            move.angular.z = self.speed_angular
            self.publisher.publish(move)
        else:
            # Stop the turtle
            stop = Twist()
            self.publisher.publish(stop)
            self.get_logger().info("Circle complete. Turtle stopped.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
