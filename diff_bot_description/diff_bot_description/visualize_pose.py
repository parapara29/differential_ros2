import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        # Subscribe to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/p3d_robot/odom',
            self.odom_callback,
            10
        )

        # Publisher for the marker topic
        self.marker_publisher = self.create_publisher(Marker, '/marker_pose', 10)

        # Initialize a Marker object for the path
        self.path_marker = Marker()
        self.path_marker.header.frame_id = "odom"
        self.path_marker.type = Marker.LINE_STRIP
        self.path_marker.action = Marker.ADD
        self.path_marker.scale.x = 0.05  # Line width
        self.path_marker.color.a = 1.0  # Alpha (transparency)
        self.path_marker.color.r = 1.0  # Red color
        self.path_marker.color.g = 0.0  # Green color
        self.path_marker.color.b = 0.0  # Blue color

    def odom_callback(self, msg):
        # Extract the position from the Odometry message
        position = msg.pose.pose.position

        # Create a Point from the position
        point = Point()
        point.x = position.x
        point.y = position.y
        point.z = position.z

        # Append the point to the path marker's points array
        self.path_marker.points.append(point)

        # Update the header timestamp
        self.path_marker.header.stamp = self.get_clock().now().to_msg()

        # Publish the marker
        self.marker_publisher.publish(self.path_marker)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
