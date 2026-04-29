#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudConverter : public rclcpp::Node
{
public:
    CloudConverter()
    : Node("cloud_converter")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map_generator/global_cloud", 10,
            std::bind(&CloudConverter::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/global_cloud", 10);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(pcl_cloud, out_msg);

        out_msg.header = msg->header;  // preserve frame + timestamp
        pub_->publish(out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};
