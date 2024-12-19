#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


class PointCloudFilterNode : public rclcpp::Node {
public:
    PointCloudFilterNode() : Node("pointcloud_filter") {
        // Subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed/zed_node/point_cloud/cloud_registered", 10,
            std::bind(&PointCloudFilterNode::pointcloudCallback, this, std::placeholders::_1)
        );

        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered/pointcloud", 10);
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert ROS2 PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);


        // Voxel Grid 필터 적용
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);
        voxel_filter.setLeafSize(0.2f, 0.2f, 0.2f);  // 리프 크기 설정
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter(*downsampled_cloud);

        // Apply Statistical Outlier Removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(downsampled_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(0.02);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);

        // Convert PCL PointCloud back to ROS2 PointCloud2
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = msg->header;

        // Publish filtered PointCloud
        pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}
