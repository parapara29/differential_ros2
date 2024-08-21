import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image, PointCloud2

class RepublisherNode(Node):
    def __init__(self):
        super().__init__('republisher_node')

        # Subscriptions
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.points_subscriber = self.create_subscription(PointCloud2, '/camera/points', self.points_callback, 10)

        # Publishers
        self.scan_publisher = self.create_publisher(LaserScan, '/republished/scan', 10)
        self.imu_publisher = self.create_publisher(Imu, '/republished/imu_data', 10)
        self.image_publisher = self.create_publisher(Image, '/republished/image_raw', 10)
        self.points_publisher = self.create_publisher(PointCloud2, '/republished/points', 10)

    def scan_callback(self, msg):
        self.scan_publisher.publish(msg)
        self.get_logger().info('Republished /scan data')

    def imu_callback(self, msg):
        self.imu_publisher.publish(msg)
        self.get_logger().info('Republished /imu_data')

    def image_callback(self, msg):
        self.image_publisher.publish(msg)
        self.get_logger().info('Republished /camera/image_raw')

    def points_callback(self, msg):
        self.points_publisher.publish(msg)
        self.get_logger().info('Republished /camera/points')


def main(args=None):
    rclpy.init(args=args)
    node = RepublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
