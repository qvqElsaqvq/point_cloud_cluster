//
// Created by elsa on 25-2-12.
//

#ifndef POINT_CLOUD_CLUSTER_HPP
#define POINT_CLOUD_CLUSTER_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


class PointCloudCluster : public rclcpp::Node
{
public:
    explicit PointCloudCluster();

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& msg);
    void cloud_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const sensor_msgs::msg::PointCloud2::ConstPtr& msg);
    void point_cloud_cut(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

private:
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_x_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_y_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_z_;
    pcl::VoxelGrid<pcl::PointXYZ> voxfilter;
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    std::string point_frame_;
    std::string prev_cloud_file_name_;
    float leaf_size_;          // 体素滤波器的体素大小
    bool use_downsample_;
    float obstacle_x_min_;     // 聚类点云范围(livox坐标系)
    float obstacle_x_max_;
    float obstacle_y_min_;
    float obstacle_y_max_;
    float obstacle_z_min_;
    float obstacle_z_max_;
    sensor_msgs::msg::PointCloud2::SharedPtr output_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;  // 创建KdTreee对象作为搜索方法
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
};

#endif //POINT_CLOUD_CLUSTER_HPP
