import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
class OdomToTransform(Node):

    def __init__(self):
        super().__init__('odom_to_transform_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/p3d_robot/odom',
            self.odometry_callback,
            10
        )
        self.timer_period = 0.1  # Set the publishing rate (in seconds)
        self.timer = self.create_timer(self.timer_period, self.publish_odom_transform)

    def odometry_callback(self, msg):
        self.last_odom_msg = msg

    def publish_odom_transform(self):
        try:
            odom_msg = self.last_odom_msg  # Get the latest stored odometry message
        except AttributeError:
            return  # No odometry message received yet

        odom_to_base_transform = TransformStamped()
        odom_to_base_transform.header.stamp = odom_msg.header.stamp
        odom_to_base_transform.header.frame_id = 'odom'
        odom_to_base_transform.child_frame_id = 'base_footprint'

        # Set the translation from odometry data
        odom_to_base_transform.transform.translation.x = odom_msg.pose.pose.position.x
        odom_to_base_transform.transform.translation.y = odom_msg.pose.pose.position.y
        odom_to_base_transform.transform.translation.z = odom_msg.pose.pose.position.z

        # Set the rotation from odometry data
        odom_to_base_transform.transform.rotation = odom_msg.pose.pose.orientation

        # Publish the transform
        self.tf_broadcaster.sendTransform(odom_to_base_transform)
    


def main(args=None):
    rclpy.init(args=args)
    odom_to_transform_publisher = OdomToTransform()
    rclpy.spin(odom_to_transform_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
