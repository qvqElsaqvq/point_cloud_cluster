//
// Created by elsa on 25-2-12.
//

#include "point_cloud_cluster/point_cloud_cluster.hpp"

PointCloudCluster::PointCloudCluster() : Node("point_cloud_cluster_node")
{
    RCLCPP_INFO(this->get_logger(), "point_cloud_cluster_node is created");
    this->declare_parameter("input_cloud_topic", "/livox/lidar_PointCloud2");
    this->declare_parameter("output_cloud_topic", "/point_cloud_cluster");
	this->declare_parameter("point_frame", "livox");
    this->declare_parameter("leaf_size", 0.1);
    this->declare_parameter("use_downsample", false);
    this->declare_parameter("obstacle_x_min", -10.0);
    this->declare_parameter("obstacle_x_max", 10.0);
    this->declare_parameter("obstacle_y_min", -10.0);
    this->declare_parameter("obstacle_y_max", 10.0);
    this->declare_parameter("obstacle_z_min", -10.0);
    this->declare_parameter("obstacle_z_max", 10.0);
    this->declare_parameter<std::string>("prev_cloud_file_name", "");
	this->declare_parameter("cut_radius", 0.001);

    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("output_cloud_topic", output_cloud_topic_);
	this->get_parameter("point_frame", point_frame_);
    this->get_parameter("leaf_size", leaf_size_);
    this->get_parameter("use_downsample", use_downsample_);
    this->get_parameter("obstacle_x_min", obstacle_x_min_);
    this->get_parameter("obstacle_x_max", obstacle_x_max_);
    this->get_parameter("obstacle_y_min", obstacle_y_min_);
    this->get_parameter("obstacle_y_max", obstacle_y_max_);
    this->get_parameter("obstacle_z_min", obstacle_z_min_);
    this->get_parameter("obstacle_z_max", obstacle_z_max_);
	this->get_parameter_or<std::string>("prev_cloud_file_name", prev_cloud_file_name_, "");
	this->get_parameter("cut_radius", cut_radius_);

    // 设置滤波器
    pass_through_filter_x_.setFilterFieldName("x");
    pass_through_filter_x_.setFilterLimits(obstacle_x_min_, obstacle_x_max_);
    pass_through_filter_x_.setFilterLimitsNegative(false);
    pass_through_filter_y_.setFilterFieldName("y");
    pass_through_filter_y_.setFilterLimits(obstacle_y_min_, obstacle_y_max_);
    pass_through_filter_y_.setFilterLimitsNegative(false);
    pass_through_filter_z_.setFilterFieldName("z");
    pass_through_filter_z_.setFilterLimits(obstacle_z_min_, obstacle_z_max_);
    pass_through_filter_z_.setFilterLimitsNegative(false);
	pass_through_filter_x_cut_.setFilterFieldName("x");
	pass_through_filter_x_cut_.setFilterLimits(obstacle_x_min_, obstacle_x_max_);
	pass_through_filter_x_cut_.setFilterLimitsNegative(false);
	pass_through_filter_y_cut_.setFilterFieldName("y");
	pass_through_filter_y_cut_.setFilterLimits(obstacle_y_min_, obstacle_y_max_);
	pass_through_filter_y_cut_.setFilterLimitsNegative(false);
	pass_through_filter_z_cut_.setFilterFieldName("z");
	pass_through_filter_z_cut_.setFilterLimits(obstacle_z_min_, obstacle_z_max_);
	pass_through_filter_z_cut_.setFilterLimitsNegative(false);
    voxfilter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
	voxfilter_prev_.setLeafSize(0.05, 0.05, 0.05);

    output_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
	cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	prev_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	cloud_cutted_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
	cloud_cut_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    pcl::io::loadPCDFile(prev_cloud_file_name_, cloudBlob);
	pcl::fromPCLPointCloud2(cloudBlob, *prev_cloud_);
	// 直通滤波
	pass_through_filter_x_.setInputCloud(prev_cloud_);
	pass_through_filter_x_.filter(*prev_cloud_);
	pass_through_filter_y_.setInputCloud(prev_cloud_);
	pass_through_filter_y_.filter(*prev_cloud_);
	pass_through_filter_z_.setInputCloud(prev_cloud_);
	pass_through_filter_z_.filter(*prev_cloud_);
	voxfilter_prev_.setInputCloud(prev_cloud_);
	voxfilter_prev_.filter(*prev_cloud_);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_,
        10, std::bind(&PointCloudCluster::pointCloudCallback, this, std::placeholders::_1));
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
	cloud_cut_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud_cut", 10);

    RCLCPP_INFO(this->get_logger(), "point_cloud_cluster_node初始化完成");
}

void PointCloudCluster::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& msg)
{
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
        voxfilter_.setInputCloud(cloud);
        voxfilter_.filter(*cloud);
    }
	RCLCPP_INFO(this->get_logger(), "降采样后点云数量为：%lu", cloud->points.size());

    tree_->setInputCloud(cloud);
    //去除先验地图部分
	point_cloud_cut(cloud, msg);
    //点云聚类
    cloud_cluster(cloud, msg);
}

void PointCloudCluster::cloud_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                      const sensor_msgs::msg::PointCloud2::ConstPtr& msg)
{
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); // 单位：m
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree_);
    ec.setInputCloud(cloud_cut_);
    //聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个组件点云的下标
    ec.extract(cluster_indices);
	RCLCPP_INFO(this->get_logger(), "聚类后种类数量为：%lu", cluster_indices.size());

    //遍历抽取结果，发布
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        //通过下标，逐个填充
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
        	cloud_->points.push_back(cloud_cut_->points[*pit]);
        }
    }
    //设置点云属性
    cloud_->width = cloud_->points.size();
    cloud_->height = 1;
    cloud_->is_dense = true;

	pcl::toROSMsg(*cloud_, *output_cloud_);
	cloud_->clear();
    output_cloud_->header.stamp = msg->header.stamp;
    output_cloud_->header.frame_id = point_frame_;
    cluster_pub_->publish(*output_cloud_);
}

void PointCloudCluster::point_cloud_cut(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                        const sensor_msgs::msg::PointCloud2::ConstPtr& msg)
{
	cloud_cut_->clear();

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(prev_cloud_);

    std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	std::vector<int> indices;

	for (size_t i = 0; i < cloud->size(); i++)
	{
		if (kdtree->radiusSearch(cloud->points[i], cut_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			indices.push_back(i);
		}
	}

	for (size_t i = 0; i < cloud->size(); i++)
	{
		if (find(indices.begin(), indices.end(), i) == indices.end())
			cloud_cut_->push_back(cloud->points[i]);
	}

	std::cout << "cloud1 has " << cloud->size() << " points" << std::endl;
	std::cout << "cloud2 has " << prev_cloud_->size() << " points" << std::endl;
	std::cout << "cloud3 has " << cloud_cut_->size() << " points" << std::endl;

	cloud_cut_->width = cloud_cut_->points.size();
	cloud_cut_->height = 1;
	cloud_cut_->is_dense = true;

	pcl::toROSMsg(*cloud_cut_, *cloud_cutted_);
	cloud_cutted_->header.stamp = msg->header.stamp;
	cloud_cutted_->header.frame_id = point_frame_;
	cloud_cut_pub_->publish(*cloud_cutted_);
}
