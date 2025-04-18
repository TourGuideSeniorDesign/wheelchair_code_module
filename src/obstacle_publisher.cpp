// Created by Angela B. on 4/17/25.

#include "obstacle_publisher.hpp"   

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <limits>        // for numeric_limits
#include <algorithm>     


ObstacleDetector::ObstacleDetector() : Node("obstacle_detector")
{
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&ObstacleDetector::cloudCallback, this, std::placeholders::_1));

  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("obstacle_detected", 10);
  RCLCPP_INFO(get_logger(), "ObstacleDetector node started.");
}


void ObstacleDetector::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS → PCL 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) return;

  // Down‑sample 
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.filter(*down);

  // Ground removal (RANSAC plane) 
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2f);

  pcl::PointIndices::Ptr ground_idx(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr dummy(new pcl::ModelCoefficients);
  seg.setInputCloud(down);
  seg.segment(*ground_idx, *dummy);

  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
  if (!ground_idx->indices.empty())
  {
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(down);
    ex.setIndices(ground_idx);
    ex.setNegative(true);            // keep everything that is NOT ground
    ex.filter(*obstacles);
  }
  else
  {
    obstacles = down;                // no plane found → treat all as obs
  }

  // 4. Denoise 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(obstacles);
  sor.setMeanK(20);
  sor.setStddevMulThresh(2.0);
  sor.filter(*obstacles);

  // 5. Euclidean clustering 
  std::vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(obstacles);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud(obstacles);
  ec.setSearchMethod(tree);
  ec.setClusterTolerance(0.2f);
  ec.setMinClusterSize(30);
  ec.extract(clusters);

  // 6. Bounding‑box distance test 
  constexpr float STOP_DIST = 1.0f;   // metres
  bool obstacle_close = false;

  for (const auto &cluster : clusters)
  {
    Eigen::Vector3f min_pt( std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());
    Eigen::Vector3f max_pt(-std::numeric_limits<float>::max(),
                           -std::numeric_limits<float>::max(),
                           -std::numeric_limits<float>::max());

    for (int idx : cluster.indices)
    {
      const auto &p = obstacles->at(idx);
      min_pt = min_pt.cwiseMin(Eigen::Vector3f(p.x, p.y, p.z));
      max_pt = max_pt.cwiseMax(Eigen::Vector3f(p.x, p.y, p.z));
    }

    float distance = (min_pt + max_pt).norm() * 0.5f;  // centre distance
    if (distance < STOP_DIST)
    {
      obstacle_close = true;
      break;
    }
  }

  // 7. Publish flag 
  std_msgs::msg::Bool msg_out;
  msg_out.data = obstacle_close;
  obstacle_pub_->publish(msg_out);

  RCLCPP_DEBUG(get_logger(), "Obstacle flag: %s",
               obstacle_close ? "TRUE" : "FALSE");
}
