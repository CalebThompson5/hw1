# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleDrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Radius, velocities
        self.speed_linear = 1.0
        self.radius_circle = 2.0
        self.speed_angular = 0.5

        # This will tell us if we have traveled the full circumference
        self.circle_duration = (2 * 3.14159 * self.radius_circle) / self.speed_linear

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.send_velocity_command)

    def send_velocity_command(self):
        current_time = self.get_clock().now()
        time_elapsed = (current_time - self.start_time).nanoseconds / 1e9

        if time_elapsed < self.circle_duration:
            move = Twist()
            move.linear.x = self.speed_linear
            move.angular.z = self.speed_angular
            self.publisher.publish(move)
        else:
            # Stop the turtle
            stop = Twist()
            self.publisher.publish(stop)
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
