//
// Created by elsa on 25-2-12.
//

#include "point_cloud_cluster/point_cloud_cluster.hpp"

PointCloudCluster::PointCloudCluster() : Node("point_cloud_cluster_node")
{
    RCLCPP_INFO(this->get_logger(), "point_cloud_cluster_node is created");
    this->declare_parameter("input_cloud_topic", "/livox/lidar_PointCloud2");
    this->declare_parameter("output_cloud_topic", "/point_cloud_cluster");
    this->declare_parameter("leaf_size", 0.1);
    this->declare_parameter("use_downsample", true);
    this->declare_parameter("obstacle_x_min", -10.0);
    this->declare_parameter("obstacle_x_max", 10.0);
    this->declare_parameter("obstacle_y_min", -10.0);
    this->declare_parameter("obstacle_y_max", 10.0);
    this->declare_parameter("obstacle_z_min", -10.0);
    this->declare_parameter("obstacle_z_max", 10.0);

    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("output_cloud_topic", output_cloud_topic_);
    this->get_parameter("leaf_size", leaf_size_);
    this->get_parameter("use_downsample", use_downsample_);
    this->get_parameter("obstacle_x_min", obstacle_x_min_);
    this->get_parameter("obstacle_x_max", obstacle_x_max_);
    this->get_parameter("obstacle_y_min", obstacle_y_min_);
    this->get_parameter("obstacle_y_max", obstacle_y_max_);
    this->get_parameter("obstacle_z_min", obstacle_z_min_);
    this->get_parameter("obstacle_z_max", obstacle_z_max_);

    // 设置滤波器的体素大小
    pass_through_filter_x_.setFilterFieldName("x");
    pass_through_filter_x_.setFilterLimits(obstacle_x_min_, obstacle_x_max_);
    pass_through_filter_x_.setFilterLimitsNegative(false);
    pass_through_filter_y_.setFilterFieldName("y");
    pass_through_filter_y_.setFilterLimits(obstacle_y_min_, obstacle_y_max_);
    pass_through_filter_y_.setFilterLimitsNegative(false);
    pass_through_filter_z_.setFilterFieldName("z");
    pass_through_filter_z_.setFilterLimits(obstacle_z_min_, obstacle_z_max_);
    pass_through_filter_z_.setFilterLimitsNegative(false);
    voxfilter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

    output_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_,
        10, std::bind(&PointCloudCluster::pointCloudCallback, this, std::placeholders::_1));
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "point_cloud_cluster_node初始化完成");
}

void PointCloudCluster::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& msg)
{
    output_cloud_->header.stamp = msg->header.stamp;
    output_cloud_->header.frame_id = msg->header.frame_id;
    // 将点云转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    // 直通滤波
    pass_through_filter_x_.setInputCloud(cloud);
    pass_through_filter_x_.filter(*cloud);
    pass_through_filter_y_.setInputCloud(cloud);
    pass_through_filter_y_.filter(*cloud);
    pass_through_filter_z_.setInputCloud(cloud);
    pass_through_filter_z_.filter(*cloud);
    // 降采样
    if (use_downsample_)
    {
        voxfilter.setInputCloud(cloud);
        voxfilter.filter(*cloud);
    }

    //点云聚类
    cloud_cluster(cloud);
}

void PointCloudCluster::cloud_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // 创建KdTreee对象作为搜索方法
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    //聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个组件点云的下标
    ec.extract(cluster_indices);

    //遍历抽取结果，发布
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        //创建临时保存点云族的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        //通过下标，逐个填充
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }

        //设置点云属性
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::toROSMsg(*cloud_cluster, *output_cloud_);
        cluster_pub_->publish(*output_cloud_);
    }
}
