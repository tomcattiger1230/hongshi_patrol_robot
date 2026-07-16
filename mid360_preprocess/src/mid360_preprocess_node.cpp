#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class Mid360Preprocess : public rclcpp::Node
{
public:
    Mid360Preprocess() : Node("mid360_preprocess")
    {
        const auto input_topic = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        const auto output_topic = this->declare_parameter<std::string>("output_topic", "/filtered_points");
        min_z_ = this->declare_parameter<double>("min_z", -0.2);
        max_z_ = this->declare_parameter<double>("max_z", 2.5);
        voxel_size_ = this->declare_parameter<double>("voxel_size", 0.05);

        // 订阅原始点云
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&Mid360Preprocess::cloudCallback, this, std::placeholders::_1));

        // 发布处理后的点云
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(), "Mid360 Preprocess Node Started");
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        // 1. 直通滤波：去除地面和天花板
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z_, max_z_);
        pass.filter(*cloud);

        // 2. 体素滤波：降采样
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(cloud);
        const auto leaf = static_cast<float>(voxel_size_);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*cloud);

        // 3. 发布
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = msg->header;
        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    double min_z_;
    double max_z_;
    double voxel_size_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mid360Preprocess>());
    rclcpp::shutdown();
    return 0;
}
