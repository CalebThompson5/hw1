# !/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DiamondDrawer(Node):
    def __init__(self):
        super().__init__('diamond_drawer')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Velocities
        self.linear_speed = 1.0
        self.angular_speed = 1.0

        # Times to move forward and turn
        self.edge_time = 2.0
        self.turn_90_time = 1.5708
        self.turn_45_time = 0.7854

        # States
        self.state = "init_turn"
        self.start_time = self.get_clock().now()
        self.edges_done = 0

        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        twist = Twist()

        if self.state == "init_turn":
            twist.angular.z = self.angular_speed
            if elapsed >= self.turn_45_time:
                self.next_state("forward")

        elif self.state == "forward":
            twist.linear.x = self.linear_speed
            if elapsed >= self.edge_time:
                self.next_state("turn")

        elif self.state == "turn":
            twist.angular.z = self.angular_speed
            if elapsed >= self.turn_90_time:
                self.edges_done += 1
                if self.edges_done < 4:
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
    node = DiamondDrawer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()