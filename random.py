# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TriangleDrawer(Node):
    def __init__(self):
        super().__init__('triangle_drawer')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Speeds
        self.linear_speed = 1.0
        self.angular_speed = 1.0

        # Motion durations (precomputed, in seconds)
        self.edge_time = 2.0 / self.linear_speed           # side length 2.0
        self.turn_120_time = 2.0944 / self.angular_speed   # 120° in radians ≈ 2.0944

        # State machine
        self.phase = "forward"
        self.start_time = self.get_clock().now()
        self.edges_done = 0

        # Check every 50 ms
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        twist = Twist()

        if self.phase == "forward":
            twist.linear.x = self.linear_speed
            if elapsed >= self.edge_time:
                self.next_phase("turn")

        elif self.phase == "turn":
            twist.angular.z = self.angular_speed
            if elapsed >= self.turn_120_time:
                self.edges_done += 1
                if self.edges_done < 3:
                    self.next_phase("forward")
                else:
                    self.next_phase("stop")

        elif self.phase == "stop":
            self.pub.publish(Twist())  # ensure stop
            self.get_logger().info("Completed one triangle — shutting down.")
            rclpy.shutdown()
            return

        self.pub.publish(twist)

    def next_phase(self, new_phase):
        self.phase = new_phase
        self.start_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = TriangleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()