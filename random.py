# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TriangleDrawer(Node):
    def __init__(self):
        super().__init__('triangle_drawer')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Velocities
        self.linear_speed = 1.0
        self.angular_speed = 1.0

        # Time to move forwad and turn
        self.edge_time = 2.0
        self.turn_120_time = 2.0944

        # States
        self.state = "forward"
        self.start_time = self.get_clock().now()
        self.edges_done = 0

        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        twist = Twist()

        if self.state == "forward":
            twist.linear.x = self.linear_speed
            if elapsed >= self.edge_time:
                self.next_state("turn")

        elif self.state == "turn":
            twist.angular.z = self.angular_speed
            if elapsed >= self.turn_120_time:
                self.edges_done += 1
                if self.edges_done < 3:
                    self.next_state("forward")
                else:
                    self.next_state("stop")

        elif self.state == "stop":
            self.pub.publish(Twist())
            rclpy.shutdown()
            return

        self.pub.publish(twist)

    def next_state(self, new_state):
        self.state = new_state
        self.start_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = TriangleDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()