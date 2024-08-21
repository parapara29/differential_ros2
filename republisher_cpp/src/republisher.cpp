#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class RepublisherNode : public rclcpp::Node
{
public:
    RepublisherNode() : Node("republisher_node")
    {
        // Subscriptions
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&RepublisherNode::scan_callback, this, std::placeholders::_1));
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_data", 10, std::bind(&RepublisherNode::imu_callback, this, std::placeholders::_1));
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&RepublisherNode::image_callback, this, std::placeholders::_1));
        points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10, std::bind(&RepublisherNode::points_callback, this, std::placeholders::_1));

        // Publishers
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/republished/scan", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/republished/imu_data", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/republished/image_raw", 10);
        points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/republished/points", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Republished /scan data");
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Republished /imu_data");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Republished /camera/image_raw");
    }

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        points_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Republished /camera/points");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RepublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
