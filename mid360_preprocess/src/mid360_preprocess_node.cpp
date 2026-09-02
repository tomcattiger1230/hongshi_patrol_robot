#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <cstdint>

class Mid360Preprocess : public rclcpp::Node
{
public:
    Mid360Preprocess() : Node("mid360_preprocess")
    {
        const auto input_topic = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        const auto output_topic = this->declare_parameter<std::string>("output_topic", "/filtered_points");
        output_frame_ = this->declare_parameter<std::string>("output_frame", "");
        min_z_ = this->declare_parameter<double>("min_z", -0.2);
        max_z_ = this->declare_parameter<double>("max_z", 2.5);
        voxel_size_ = this->declare_parameter<double>("voxel_size", 0.05);
        self_filter_enabled_ = this->declare_parameter<bool>("self_filter_enabled", true);
        lidar_x_ = this->declare_parameter<double>("lidar_x", 0.365);
        self_min_x_ = this->declare_parameter<double>("self_min_x", -0.82);
        self_max_x_ = this->declare_parameter<double>("self_max_x", 0.82);
        self_min_y_ = this->declare_parameter<double>("self_min_y", -0.485);
        self_max_y_ = this->declare_parameter<double>("self_max_y", 0.485);

        // 订阅原始点云
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(),
            std::bind(&Mid360Preprocess::cloudCallback, this, std::placeholders::_1));

        // 发布处理后的点云
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(10));

        RCLCPP_INFO(
            this->get_logger(),
            "Mid360 Preprocess Node Started (output frame: %s)",
            output_frame_.empty() ? "<input frame>" : output_frame_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "Self filter: %s; base_link x=[%.3f, %.3f], y=[%.3f, %.3f], lidar_x=%.3f",
            self_filter_enabled_ ? "enabled" : "disabled",
            self_min_x_, self_max_x_, self_min_y_, self_max_y_, lidar_x_);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Navigation only needs geometry. Isaac RTX point clouds do not carry
        // an intensity field, so use PointXYZ for both real and simulated
        // clouds instead of logging a conversion warning for every frame.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // 1. 直通滤波：去除地面和天花板
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z_, max_z_);
        pass.filter(*cloud);

        // 2. 去除车体自身反射。点坐标位于雷达坐标系；当前实车标定无旋转，
        // 雷达位于 base_link 前方 lidar_x 米。此步骤必须在体素降采样前执行。
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_input = cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr without_self;
        if (self_filter_enabled_) {
            without_self.reset(new pcl::PointCloud<pcl::PointXYZ>());
            without_self->reserve(cloud->size());
            for (const auto & point : cloud->points) {
                const double base_x = static_cast<double>(point.x) + lidar_x_;
                const bool inside_vehicle =
                    base_x >= self_min_x_ && base_x <= self_max_x_ &&
                    static_cast<double>(point.y) >= self_min_y_ &&
                    static_cast<double>(point.y) <= self_max_y_;
                if (!inside_vehicle) {
                    without_self->push_back(point);
                }
            }
            without_self->width = static_cast<std::uint32_t>(without_self->size());
            without_self->height = 1;
            without_self->is_dense = cloud->is_dense;
            voxel_input = without_self;
        }

        // 3. 体素滤波：降采样
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(voxel_input);
        const auto leaf = static_cast<float>(voxel_size_);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*cloud);

        // 4. 发布
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = msg->header;
        if (!output_frame_.empty()) {
            // Gazebo may publish its scoped sensor path (for example
            // patrol_robot/base_footprint/mid360s) instead of the URDF link
            // name. The points are already expressed in the lidar sensor
            // coordinates, so normalizing only the header makes the cloud
            // match the robot_state_publisher TF tree.
            output.header.frame_id = output_frame_;
        }
        pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::string output_frame_;
    double min_z_;
    double max_z_;
    double voxel_size_;
    bool self_filter_enabled_;
    double lidar_x_;
    double self_min_x_;
    double self_max_x_;
    double self_min_y_;
    double self_max_y_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mid360Preprocess>());
    rclcpp::shutdown();
    return 0;
}
