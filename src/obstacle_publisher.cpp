// obstacle_publisher.cpp
// Updated 2025-04-23 — rectangular keep-out zone added
// Created by Angela B. on 4/17/25.

#include "obstacle_publisher.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>

#include <Eigen/Core>
#include <visualization_msgs/msg/marker_array.hpp>

#include <limits>
#include <algorithm>
#include <cmath>
#include <cfloat>  // FLT_MAX

// ───────────────────────────────────────────────────────────────
ObstaclePublisher::ObstaclePublisher() : Node("obstacle_publisher")
{
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&ObstaclePublisher::cloudCallback, this, std::placeholders::_1));

  obstacle_flag_pub_   = create_publisher<std_msgs::msg::Bool>("obstacle_detected", 10);
  obstacles_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("obstacles_cloud", 10);
  markers_pub_         = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);

  RCLCPP_INFO(get_logger(), "ObstaclePublisher node started.");
}

// ───────────────────────────────────────────────────────────────
void ObstaclePublisher::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert to PCL
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) return;

  // 1. Down-sample
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.filter(*down);

  // 2. Remove rider (crop box around torso, keep everything else)
  {
    pcl::CropBox<pcl::PointXYZ> torso;
    torso.setMin(Eigen::Vector4f(-0.35f, -0.35f, -0.70f, 1));
    torso.setMax(Eigen::Vector4f( 0.35f,  0.35f,  0.40f, 1));
    torso.setNegative(true);
    torso.setInputCloud(down);
    torso.filter(*down);
  }

  // 3. Remove ground plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
  {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2f);

    pcl::PointIndices::Ptr ground_idx(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    seg.setInputCloud(down);
    seg.segment(*ground_idx, *coeff);

    bool floor_plane = false;
    if (!coeff->values.empty()) {
      Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
      n.normalize();
      floor_plane = std::acos(std::abs(n.z())) * 180.0f / M_PI < 15.0f;
    }

    if (floor_plane && !ground_idx->indices.empty()) {
      pcl::ExtractIndices<pcl::PointXYZ> ex;
      ex.setInputCloud(down);
      ex.setIndices(ground_idx);
      ex.setNegative(true);
      ex.filter(*obstacles);
    } else {
      obstacles = down;
    }
  }

  // 4. Denoise
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(obstacles);
  sor.setMeanK(20);
  sor.setStddevMulThresh(2.0);
  sor.filter(*obstacles);

  if (obstacles->empty()) {
    std_msgs::msg::Bool b;
    b.data = false;
    obstacle_flag_pub_->publish(b);
    return;
  }

  // 5. Clustering — for visualisation only
  std::vector<pcl::PointIndices> clusters;
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(obstacles);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(obstacles);
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(0.2f);
    ec.setMinClusterSize(10);
    ec.extract(clusters);
  }

  // 6. Rectangular keep-out zone test
  constexpr float STOP_X_FWD  = 0.00f;  // start just ahead of bumper
  constexpr float STOP_X_BACK = 0.60f;  // 60 cm forward
  constexpr float STOP_Y_HALF = 0.45f;  // 90 cm total width (wheelbase + margin)

  bool close_now = false;
  for (const auto &p : obstacles->points) {
    if (p.z < -0.1f || p.z > 1.5f) continue;  // height gate
    if (p.x > STOP_X_FWD && p.x < STOP_X_BACK &&
        std::fabs(p.y) < STOP_Y_HALF) {
      close_now = true;
      break;
    }
  }

  // 6b. Debounce (needs 5 hits to stop, 3 clears to go)
  static int  hit_cnt = 0;
  static int  clr_cnt = 0;
  static bool latched = false;

  if (close_now) { hit_cnt++; clr_cnt = 0; }
  else           { clr_cnt++; hit_cnt = 0; }

  if (!latched && hit_cnt >= 5) latched = true;
  if (latched  && clr_cnt >= 3) latched = false;

  // 7a. Publish Boolean obstacle flag
  std_msgs::msg::Bool flag_msg;
  flag_msg.data = latched;
  obstacle_flag_pub_->publish(flag_msg);

  // 7b. Publish filtered cloud
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*obstacles, cloud_msg);
  cloud_msg.header = msg->header;
  obstacles_cloud_pub_->publish(cloud_msg);

  // 7c. Publish bounding-box markers
  visualization_msgs::msg::MarkerArray boxes;
  int id = 0;
  for (const auto &cl : clusters) {
    Eigen::Vector3f min_pt( FLT_MAX,  FLT_MAX,  FLT_MAX);
    Eigen::Vector3f max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (int idx : cl.indices) {
      const auto &p = obstacles->at(idx);
      Eigen::Vector3f pt(p.x, p.y, p.z);
      min_pt = min_pt.cwiseMin(pt);
      max_pt = max_pt.cwiseMax(pt);
    }

    visualization_msgs::msg::Marker m;
    m.header = msg->header;
    m.ns     = "obstacle_boxes";
    m.id     = id++;
    m.type   = m.CUBE;
    m.action = m.ADD;

    m.pose.position.x = 0.5f * (min_pt.x() + max_pt.x());
    m.pose.position.y = 0.5f * (min_pt.y() + max_pt.y());
    m.pose.position.z = 0.5f * (min_pt.z() + max_pt.z());
    m.pose.orientation.w = 1.0;

    m.scale.x = (max_pt.x() - min_pt.x()) + 0.10f;
    m.scale.y = (max_pt.y() - min_pt.y()) + 0.10f;
    m.scale.z = (max_pt.z() - min_pt.z()) + 0.10f;

    m.color.r = 1.0f;
    m.color.g = 0.6f;
    m.color.b = 0.0f;
    m.color.a = 0.8f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);

    boxes.markers.push_back(m);
  }

  markers_pub_->publish(boxes);
}
