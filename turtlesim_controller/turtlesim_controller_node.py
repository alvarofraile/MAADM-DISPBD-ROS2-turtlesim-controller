#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Char
from geometry_msgs.msg import Twist

class TurtlesimController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')

        self.speed = 0.5
        self.rotation_speed = 0.5

        self.char_subscriber = self.create_subscription(Char, "charstream", self.process_action, 10)
        self.ts_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Controlador Turtlesim creado")

    def process_action(self, msg):
        character = str(msg.data)
        twist = Twist()
        movement = 0.0
        rotation = 0.0

        if character in ["119", "87"]:
            movement += self.speed
            self.ts_publisher.publish(twist)
            self.get_logger().info("Forward")
        if character in ["115", "83"]:
            movement += -self.speed
            self.ts_publisher.publish(twist)
            self.get_logger().info("Backward")
        if character in ["97", "65"]:
            rotation += self.rotation_speed
            self.ts_publisher.publish(twist)
            self.get_logger().info("Rotate Left")
        if character in ["100", "68"]:
            rotation += -self.rotation_speed
            self.get_logger().info("Rotate Right")
        if character in ["32"]:
            self.toggle_drawing()
        if character in ["99", "67"]:
            self.clear_drawn()
        if character in ["114", "82"]:
            self.reset_position()

        twist.linear.x = movement
        twist.angular.z = rotation

        self.ts_publisher.publish(twist)

    def toggle_drawing(self):
        pass

    def clear_drawn(self):
        pass

    def reset_position(self):
        pass

def main(args=None):
    try:
        rclpy.init(args=args)
        turtlesim_controller = TurtlesimController()
        rclpy.spin(turtlesim_controller)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()