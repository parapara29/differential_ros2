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
        self.joint_state_velocity.name = ["left_joint", "right_joint"]  # These are the names of your differential drive joints

    def twist_callback(self, msg):
        # Extract the forward (linear) and rotational (angular) velocities from the Twist message
        linear_velocity = msg.linear.x  # Forward velocity
        angular_velocity = msg.angular.z  # Rotational velocity

        # Robot parameters
        wheel_radius = 0.03  # Radius of the wheels (in meters)
        wheel_base = 0.520  # Distance between the wheels (in meters)

        # Calculate the velocity for each wheel
        left_wheel_velocity = (linear_velocity - angular_velocity * wheel_base / 2) / wheel_radius
        right_wheel_velocity = (linear_velocity + angular_velocity * wheel_base / 2) / wheel_radius

        # Set the joint state velocities
        self.joint_state_velocity.velocity = [left_wheel_velocity, right_wheel_velocity]

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
