#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


class PositionVelocityPublisher(Node):
    def __init__(self):
        super().__init__('position_velocity_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        self.subscription_ = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.joint_state_velocity = JointState()
        self.joint_state_velocity.name = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4"]

    def twist_callback(self, msg):
        # Convert twist message to joint state velocities
        u = msg.linear.x
        v = msg.linear.y
        r = msg.angular.z

        rim_radius_ = 0.03
        rim_separation_length_ = 0.520
        rim_separation_width_ = 0.610
        self.joint_state_velocity.velocity = []

        # Calculate joint state velocities based on twist values
        self.joint_state_velocity.velocity.append(
            (1 / rim_radius_) * (u + v + r * ((rim_separation_length_ - rim_separation_width_) / 2))
        )
        self.joint_state_velocity.velocity.append(
            (1 / rim_radius_) * (u - v + r * ((rim_separation_length_ - rim_separation_width_) / 2))
        )
        self.joint_state_velocity.velocity.append(
            (1 / rim_radius_) * (u + v - r * ((rim_separation_length_ - rim_separation_width_) / 2))
        )
        self.joint_state_velocity.velocity.append(
            (1 / rim_radius_) * (u - v - r * ((rim_separation_length_ - rim_separation_width_) / 2))
        )

        # Publish joint state velocities
        self.publisher_.publish(self.joint_state_velocity)


def main(args=None):
    rclpy.init(args=args)
    position_velocity_publisher = PositionVelocityPublisher()
    rclpy.spin(position_velocity_publisher)
    position_velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
