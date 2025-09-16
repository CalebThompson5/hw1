# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class RectangleDrawer(Node):
    def __init__(self):
        super().__init__('square_once')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Parameters
        self.side_length = 2.0       # meters
        self.linear_speed = 1.0      # m/s
        self.angular_speed = math.pi / 2  # rad/s (90 deg/s turn)

        # Durations
        self.forward_time = self.side_length / self.linear_speed
        self.turn_time = (3.14159/ 2) / self.angular_speed

        # State
        self.start_time = self.get_clock().now()
        self.state = "forward"
        self.sides_completed = 0

        # Timer
        self.timer = self.create_timer(0.1, self.run_square)

    def run_square(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        msg = Twist()

        if self.sides_completed >= 4:
            # Finished the square
            self.get_logger().info("Square complete! Stopping.")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.timer.cancel()
            return

        if self.state == "forward":
            if elapsed_time < self.forward_time:
                msg.linear.x = self.linear_speed
                msg.angular.z = 0.0
            else:
                # Transition to turning
                self.state = "turn"
                self.start_time = self.get_clock().now()
        elif self.state == "turn":
            if elapsed_time < self.turn_time:
                msg.linear.x = 0.0
                msg.angular.z = self.angular_speed
            else:
                # Transition back to forward
                self.state = "forward"
                self.start_time = self.get_clock().now()
                self.sides_completed += 1

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RectangleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
