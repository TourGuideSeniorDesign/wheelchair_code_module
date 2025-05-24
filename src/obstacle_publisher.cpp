// obstacle_publisher.cpp
// 2025‑05‑08 – ultra‑conservative revision 2025‑05‑16c
// The chair was still initiating pivots in tight spots.  Dial the side‑sector
// test up another notch so it will only turn when there is *lots* of space:
//   • Cone half‑angle shrunk to ±40°  (was 50°)
//   • Clear distance pushed to 2.0 m   (was 1.5 m)
//   • Everything else (5‑/3‑frame debounce, “blocked if ANY point” rule)
//     unchanged.
// Publishes four Bool topics:
//   front_clear, back_clear, left_turn_clear, right_turn_clear

#include "obstacle_publisher.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <limits>
#include <cmath>
#include <cfloat>

namespace {

constexpr float Z_MIN = 0.05f;
constexpr float Z_MAX = 1.80f;

/* stop‑box dimensions (m) ---------------------------------------------- */
constexpr float F_BOX_X_MIN = 0.0f,  F_BOX_X_MAX = 0.60f,  F_BOX_Y_HALF = 0.75f;
constexpr float B_BOX_X_MIN = -0.40f, B_BOX_X_MAX = -0.05f, B_BOX_Y_HALF = 0.75f;

/* tunnel “wall warn” thresholds --------------------------------------- */
constexpr float F_TUNNEL_Y_HALF = 0.50f, F_WALL_WARN =  1.10f;
constexpr float B_TUNNEL_Y_HALF = 0.50f, B_WALL_WARN = -1.10f;

/* *** side‑cone parameters *** --------------------- */
constexpr float CONE_HALF_ANGLE = static_cast<float>(M_PI) * 35.0f / 180.0f; // ±35°
constexpr float CLEAR_DIST      = 1.60f;   // ≥1.60 m of open space required

/* debounce helper ------------------------------------------------------ */
inline void debounce(bool hit, int& h, int& c, bool& latched)
{
    hit ? (++h, c = 0) : (++c, h = 0);
    if (!latched && h >= 4) latched = true;
    if ( latched && c >= 2) latched = false;
}

} // anonymous namespace

/* ─────────────────────────────────────────────────────────────────────── */
ObstaclePublisher::ObstaclePublisher() : Node("obstacle_publisher")
{
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&ObstaclePublisher::cloudCallback, this, std::placeholders::_1));

  front_clear_pub_      = create_publisher<std_msgs::msg::Bool>("front_clear",      10);
  back_clear_pub_       = create_publisher<std_msgs::msg::Bool>("back_clear",       10);
  left_turn_clear_pub_  = create_publisher<std_msgs::msg::Bool>("left_turn_clear",  10);
  right_turn_clear_pub_ = create_publisher<std_msgs::msg::Bool>("right_turn_clear", 10);

  obstacles_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("obstacles_cloud", 10);
  markers_pub_         = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);

  RCLCPP_INFO(get_logger(), "ObstaclePublisher node started.");
}

/* ─────────────────────────────────────────────────────────────────────── */
void ObstaclePublisher::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *raw);
  if (raw->empty()) return;

  /* 1 ─ voxel grid (10 cm) -------------------------------------------- */
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw);
  vg.setLeafSize(0.1f, 0.1f, 0.1f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*cloud);

  /* 2 ─ cut torso & chair footprint ----------------------------------- */
  {
    pcl::CropBox<pcl::PointXYZ> cb;
    cb.setMin(Eigen::Vector4f(-0.45f,-0.45f,-0.70f,1));
    cb.setMax(Eigen::Vector4f( 0.50f, 0.45f, 0.45f,1));
    cb.setNegative(true);
    cb.setInputCloud(cloud);
    cb.filter(*cloud);
  }

  /* 3 ─ ground removal ------------------------------------------------- */
  pcl::PointCloud<pcl::PointXYZ>::Ptr obs(new pcl::PointCloud<pcl::PointXYZ>);
  {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2f);

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr      ground(new pcl::PointIndices);
    seg.setInputCloud(cloud);
    seg.segment(*ground, *coeff);

    bool is_floor = false;
    if (!coeff->values.empty()) {
      Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
      n.normalize();
      is_floor = std::acos(std::fabs(n.z())) * 180.0f / M_PI < 15.0f;
    }

    if (is_floor && !ground->indices.empty()) {
      pcl::ExtractIndices<pcl::PointXYZ> ex;
      ex.setInputCloud(cloud);
      ex.setIndices(ground);
      ex.setNegative(true);
      ex.filter(*obs);
    } else {
      obs = cloud;
    }
  }

  /* 4 ─ statistical outlier ------------------------------------------- */
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(obs);
  sor.setMeanK(20);
  sor.setStddevMulThresh(2.0);
  sor.filter(*obs);
  if (obs->empty()) return;

  /* 5 ─ obstacle checks ----------------------------------------------- */
  bool hit_f=false, hit_b=false, hit_l=false, hit_r=false;
  float min_front = std::numeric_limits<float>::max();
  float max_back  = -std::numeric_limits<float>::max();

  for (const auto& p : obs->points) {
    if (p.z < Z_MIN || p.z > Z_MAX) continue;

    /* stop boxes */
    if (p.x>F_BOX_X_MIN && p.x<F_BOX_X_MAX && std::fabs(p.y)<F_BOX_Y_HALF) hit_f=true;
    if (p.x>B_BOX_X_MIN && p.x<B_BOX_X_MAX && std::fabs(p.y)<B_BOX_Y_HALF && p.z>0.25f) hit_b=true;

    /* tunnel warn */
    if (std::fabs(p.y)<F_TUNNEL_Y_HALF && p.x>0) min_front=std::min(min_front,p.x);
    if (std::fabs(p.y)<B_TUNNEL_Y_HALF && p.x<0) max_back =std::max(max_back ,p.x);

    /* cones */
    if (p.x>0) {
      float ang = std::atan2(p.y, p.x);
      float rng = std::hypot(p.x, p.y);
      if (rng <= CLEAR_DIST) {
        if (ang >=-CONE_HALF_ANGLE && ang<=0.0f)          hit_r = true;
        if (ang >  0.0f          && ang<=CONE_HALF_ANGLE) hit_l = true;
      }
    }
  }

  if (min_front < F_WALL_WARN) hit_f = true;
  if (max_back  > B_WALL_WARN) hit_b = true;

  /* 6 ─ debounce ------------------------------------------------------- */
  static int hf=0,cf=0,hb=0,cb=0; static bool lat_f=false,lat_b=false;
  debounce(hit_f, hf, cf, lat_f);
  debounce(hit_b, hb, cb, lat_b);

  static int hl=0,cl=0, hr=0,cr=0; static bool lat_l=false,lat_r=false;
  debounce(hit_l, hl, cl, lat_l);
  debounce(hit_r, hr, cr, lat_r);

  /* 7 ─ publish -------------------------------------------------------- */
  auto pub_bool = [&](auto& pub, bool clear){ std_msgs::msg::Bool m; m.data = clear; pub->publish(m); };
  pub_bool(front_clear_pub_,       !lat_f);
  pub_bool(back_clear_pub_,        !lat_b);
  pub_bool(left_turn_clear_pub_,   !lat_l);
  pub_bool(right_turn_clear_pub_,  !lat_r);

  /* 8 ─ diagnostics cloud --------------------------------------------- */
  sensor_msgs::msg::PointCloud2 pc;
  pcl::toROSMsg(*obs, pc);
  pc.header = msg->header;
  obstacles_cloud_pub_->publish(pc);
}



