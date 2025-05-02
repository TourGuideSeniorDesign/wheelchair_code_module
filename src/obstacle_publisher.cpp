// obstacle_publisher.cpp
// Updated 2025-05-01  (fix rear false-positive)
//   • Chair footprint crop (removes wheels/frame)
//   • Rear keep-out box z-filter (z > 0.25 m)
//   • Correct tunnel-wall test (max_back < B_WALL_WARN)

#include "obstacle_publisher.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>
#include <visualization_msgs/msg/marker_array.hpp>
#include <limits>
#include <algorithm>
#include <cfloat>

ObstaclePublisher::ObstaclePublisher() : Node("obstacle_publisher")
{
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&ObstaclePublisher::cloudCallback, this, std::placeholders::_1));

  front_clear_pub_ = create_publisher<std_msgs::msg::Bool>("front_clear", 10);
  back_clear_pub_  = create_publisher<std_msgs::msg::Bool>("back_clear",  10);

  obstacles_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("obstacles_cloud", 10);
  markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);

  RCLCPP_INFO(get_logger(), "ObstaclePublisher node started.");
}

void ObstaclePublisher::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  /* 0. Convert */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) return;

  /* 1. Down-sample */
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
  voxel.filter(*down);

  /* 2. Remove torso */
  {
    pcl::CropBox<pcl::PointCloud<pcl::PointXYZ>::PointType> torso;
    torso.setMin(Eigen::Vector4f(-0.35f, -0.35f, -0.70f, 1));
    torso.setMax(Eigen::Vector4f( 0.35f,  0.35f,  0.20f, 1));
    torso.setNegative(true);
    torso.setInputCloud(down);
    torso.filter(*down);
  }

  /* 2b. Remove wheelchair footprint */
  {
    pcl::CropBox<pcl::PointCloud<pcl::PointXYZ>::PointType> body;
    body.setMin(Eigen::Vector4f(-0.45f, -0.45f, -0.10f, 1));  // rear 45 cm
    body.setMax(Eigen::Vector4f( 0.50f,  0.45f,  0.45f, 1));  // front 50 cm
    body.setNegative(true);               // keep outside this box
    body.setInputCloud(down);
    body.filter(*down);
  }

  /* 3. Ground removal */
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

    bool floor = false;
    if (!coeff->values.empty()) {
      Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
      n.normalize();
      floor = std::acos(std::abs(n.z())) * 180.0f / M_PI < 15.0f;
    }
    if (floor && !ground_idx->indices.empty()) {
      pcl::ExtractIndices<pcl::PointXYZ> ex;
      ex.setInputCloud(down);
      ex.setIndices(ground_idx);
      ex.setNegative(true);
      ex.filter(*obstacles);
    } else {
      obstacles = down;
    }
  }

  /* 4. Denoise */
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(obstacles);
  sor.setMeanK(20);
  sor.setStddevMulThresh(2.0);
  sor.filter(*obstacles);
  if (obstacles->empty()) return;

  /* 5. Obstacle logic */
  constexpr float Z_MIN = 0.05f, Z_MAX = 1.80f;

  /* front */
  constexpr float F_BOX_X_MIN = 0.0f,  F_BOX_X_MAX = 0.60f,  F_BOX_Y_HALF = 0.75f;
  constexpr float F_TUNNEL_Y_HALF = 0.50f, F_WALL_WARN = 1.10f;

  /* back (tight) */
  constexpr float B_BOX_X_MIN = -0.40f, B_BOX_X_MAX = -0.05f, B_BOX_Y_HALF = 0.75f;
  constexpr float B_TUNNEL_Y_HALF = 0.50f, B_WALL_WARN = -1.10f;

  bool hit_front = false, hit_back = false;
  float min_front = std::numeric_limits<float>::max();
  float max_back  = -std::numeric_limits<float>::max();

  for (const auto &p : obstacles->points) {
    if (p.z < Z_MIN || p.z > Z_MAX) continue;

    /* boxes */
    if (p.x > F_BOX_X_MIN && p.x < F_BOX_X_MAX && std::fabs(p.y) < F_BOX_Y_HALF)
      hit_front = true;

    if (p.x > B_BOX_X_MIN && p.x < B_BOX_X_MAX &&
        std::fabs(p.y) < B_BOX_Y_HALF && p.z > 0.25f)     // ignore very low hits
      hit_back = true;

    /* tunnels */
    if (std::fabs(p.y) < F_TUNNEL_Y_HALF && p.x > 0.0f)
      min_front = std::min(min_front, p.x);

    if (std::fabs(p.y) < B_TUNNEL_Y_HALF && p.x < 0.0f)
      max_back = std::max(max_back, p.x);
  }
  if (min_front < F_WALL_WARN) hit_front = true;
  if (max_back  > B_WALL_WARN) hit_back  = true;   // correct sign

  /* 6. Debounce (5 hits / 3 clears) */
  auto debounce = [](bool hit, int &h, int &c, bool &lat) {
    hit ? (++h, c = 0) : (++c, h = 0);
    if (!lat && h >= 5) lat = true;
    if ( lat && c >= 3) lat = false;
  };
  static int hf=0, cf=0, hb=0, cb=0;
  static bool lat_f=false, lat_b=false;
  debounce(hit_front, hf, cf, lat_f);
  debounce(hit_back , hb, cb, lat_b);

  /* 7. Publish flags */
  std_msgs::msg::Bool f_msg; f_msg.data = !lat_f;
  std_msgs::msg::Bool b_msg; b_msg.data = !lat_b;
  front_clear_pub_->publish(f_msg);
  back_clear_pub_->publish(b_msg);

  /* 8. Diagnostics */
  sensor_msgs::msg::PointCloud2 pc_msg;
  pcl::toROSMsg(*obstacles, pc_msg);
  pc_msg.header = msg->header;
  obstacles_cloud_pub_->publish(pc_msg);

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
  visualization_msgs::msg::MarkerArray boxes;
  int id = 0;
  for (const auto &cl : clusters) {
    Eigen::Vector3f mn(FLT_MAX,FLT_MAX,FLT_MAX), mx(-FLT_MAX,-FLT_MAX,-FLT_MAX);
    for (int idx : cl.indices) {
      const auto &p = obstacles->at(idx);
      Eigen::Vector3f pt(p.x,p.y,p.z);
      mn = mn.cwiseMin(pt);  mx = mx.cwiseMax(pt);
    }
    visualization_msgs::msg::Marker m;
    m.header = msg->header;  m.ns = "obstacle_boxes";  m.id = id++;
    m.type = m.CUBE;  m.action = m.ADD;  m.pose.orientation.w = 1.0;
    m.pose.position.x = 0.5f*(mn.x()+mx.x());
    m.pose.position.y = 0.5f*(mn.y()+mx.y());
    m.pose.position.z = 0.5f*(mn.z()+mx.z());
    m.scale.x = (mx.x()-mn.x())+0.10f;
    m.scale.y = (mx.y()-mn.y())+0.10f;
    m.scale.z = (mx.z()-mn.z())+0.10f;
    m.color.r = 1.0f; m.color.g = 0.6f; m.color.b = 0.0f; m.color.a = 0.8f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    boxes.markers.push_back(m);
  }
  markers_pub_->publish(boxes);
}
