// obstacle_publisher.cpp
// 2025-05-08  —  adds distance-based cone test so /left_turn_clear and
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

/* stop-box dimensions (m) */
constexpr float F_BOX_X_MIN = 0.0f,  F_BOX_X_MAX = 0.60f,  F_BOX_Y_HALF = 0.75f;
constexpr float B_BOX_X_MIN = -0.40f, B_BOX_X_MAX = -0.05f, B_BOX_Y_HALF = 0.75f;

/* tunnel “wall warn” thresholds */
constexpr float F_TUNNEL_Y_HALF = 0.50f, F_WALL_WARN =  1.10f;
constexpr float B_TUNNEL_Y_HALF = 0.50f, B_WALL_WARN = -1.10f;

/* cone fan parameters */
constexpr float CONE_HALF_ANGLE = static_cast<float>(M_PI) * 75.0f / 180.0f; // ±75°
constexpr float CLEAR_DIST      = 0.80f;   // ≥80 cm ahead in the cone ⇒ clear

/* debouncer helper */
inline void debounce(bool hit, int& h, int& c, bool& latched)
{
    hit ? (++h, c = 0) : (++c, h = 0);
    if (!latched && h >= 5) latched = true;
    if ( latched && c >= 3) latched = false;
}

} // anonymous namespace

/* ───────────────────────────────────────────────────────────────────────── */
ObstaclePublisher::ObstaclePublisher() : Node("obstacle_publisher")
{
  /* LiDAR input */
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&ObstaclePublisher::cloudCallback, this, std::placeholders::_1));

  /* Boolean flags */
  front_clear_pub_       = create_publisher<std_msgs::msg::Bool>("front_clear",       10);
  back_clear_pub_        = create_publisher<std_msgs::msg::Bool>("back_clear",        10);
  left_turn_clear_pub_   = create_publisher<std_msgs::msg::Bool>("left_turn_clear",   10);
  right_turn_clear_pub_  = create_publisher<std_msgs::msg::Bool>("right_turn_clear",  10);

  /* Diagnostics */
  obstacles_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("obstacles_cloud", 10);
  markers_pub_         = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);

  RCLCPP_INFO(get_logger(), "ObstaclePublisher node started.");
}

void ObstaclePublisher::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  /* convert to PCL */
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *raw);
  if (raw->empty()) return;

  /* voxel down-sample (10 cm) */
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(raw);
  vg.setLeafSize(0.1f, 0.1f, 0.1f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*cloud);

  /* remove torso box */
  {
    pcl::CropBox<pcl::PointXYZ> cb;
    cb.setMin(Eigen::Vector4f(-0.35f,-0.35f,-0.70f,1));
    cb.setMax(Eigen::Vector4f( 0.35f, 0.35f, 0.20f,1));
    cb.setNegative(true);
    cb.setInputCloud(cloud);
    cb.filter(*cloud);
  }

  /* remove chair footprint */
  {
    pcl::CropBox<pcl::PointXYZ> cb;
    cb.setMin(Eigen::Vector4f(-0.45f,-0.45f,-0.10f,1));
    cb.setMax(Eigen::Vector4f( 0.50f, 0.45f, 0.45f,1));
    cb.setNegative(true);
    cb.setInputCloud(cloud);
    cb.filter(*cloud);
  }

  /* RANSAC ground removal (≤15° tilt) */
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

  /* statistical outlier filter */
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(obs);
  sor.setMeanK(20);
  sor.setStddevMulThresh(2.0);
  sor.filter(*obs);
  if (obs->empty()) return;

  /* obstacle + cone logic */
  bool hit_front=false, hit_back=false;
  float min_front = std::numeric_limits<float>::max();
  float max_back  = -std::numeric_limits<float>::max();

  /* smallest range inside each cone */
  float min_r = std::numeric_limits<float>::max();   // ahead-right
  float min_l = std::numeric_limits<float>::max();   // ahead-left

  for (const auto& p : obs->points) {
    if (p.z < Z_MIN || p.z > Z_MAX) continue;

    /* stop boxes */
    if (p.x>F_BOX_X_MIN && p.x<F_BOX_X_MAX && std::fabs(p.y)<F_BOX_Y_HALF) hit_front=true;
    if (p.x>B_BOX_X_MIN && p.x<B_BOX_X_MAX && std::fabs(p.y)<B_BOX_Y_HALF && p.z>0.25f) hit_back=true;

    /* tunnel-wall warn */
    if (std::fabs(p.y)<F_TUNNEL_Y_HALF && p.x>0) min_front=std::min(min_front,p.x);
    if (std::fabs(p.y)<B_TUNNEL_Y_HALF && p.x<0) max_back =std::max(max_back ,p.x);

    /* cone fan ranges */
    if (p.x>0) {                                 // only look forward
      float ang = std::atan2(p.y, p.x);          // +y left, –y right
      float rng = std::hypot(p.x, p.y);
      if (ang >=-CONE_HALF_ANGLE && ang<=0.0f)   min_r = std::min(min_r, rng);
      if (ang > 0.0f          && ang<=CONE_HALF_ANGLE) min_l = std::min(min_l, rng);
    }
  }

  if (min_front < F_WALL_WARN) hit_front = true;
  if (max_back  > B_WALL_WARN) hit_back  = true;

  bool right_sector_blocked = (min_r < CLEAR_DIST);
  bool left_sector_blocked  = (min_l < CLEAR_DIST);

  /* debounce front/back flags */
  static int hf=0,cf=0,hb=0,cb=0; static bool lat_f=false,lat_b=false;
  debounce(hit_front,hf,cf,lat_f);
  debounce(hit_back ,hb,cb,lat_b);

  /* publish clear/blocked flags */
  auto pub_bool = [&](auto pub, bool clear) {
      std_msgs::msg::Bool m; m.data = clear; pub->publish(m);
  };
  pub_bool(front_clear_pub_ , !lat_f);
  pub_bool(back_clear_pub_  , !lat_b);
  pub_bool(left_turn_clear_pub_ , !left_sector_blocked);
  pub_bool(right_turn_clear_pub_, !right_sector_blocked);

  /* diagnostics cloud */
  sensor_msgs::msg::PointCloud2 pc;
  pcl::toROSMsg(*obs, pc);
  pc.header = msg->header;
  obstacles_cloud_pub_->publish(pc);

  /* cluster BBoxes */
  std::vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(obs);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setInputCloud(obs);
  ec.setSearchMethod(tree);
  ec.setClusterTolerance(0.2f);
  ec.setMinClusterSize(10);
  ec.extract(clusters);

  visualization_msgs::msg::MarkerArray marr;
  int id=0;
  for(const auto& cl:clusters){
      Eigen::Vector3f mn(FLT_MAX,FLT_MAX,FLT_MAX), mx(-FLT_MAX,-FLT_MAX,-FLT_MAX);
      for(int idx:cl.indices){
          const auto& p=obs->at(idx);
          Eigen::Vector3f v(p.x,p.y,p.z);
          mn=mn.cwiseMin(v); mx=mx.cwiseMax(v);
      }
      visualization_msgs::msg::Marker m;
      m.header=msg->header; m.ns="obstacle_boxes"; m.id=id++;
      m.type=m.CUBE; m.action=m.ADD; m.pose.orientation.w=1.0;
      m.pose.position.x=0.5f*(mn.x()+mx.x());
      m.pose.position.y=0.5f*(mn.y()+mx.y());
      m.pose.position.z=0.5f*(mn.z()+mx.z());
      m.scale.x=(mx.x()-mn.x())+0.10f;
      m.scale.y=(mx.y()-mn.y())+0.10f;
      m.scale.z=(mx.z()-mn.z())+0.10f;
      m.color.r=1.0f; m.color.g=0.6f; m.color.b=0.0f; m.color.a=0.8f;
      m.lifetime=rclcpp::Duration::from_seconds(0.5);
      marr.markers.push_back(m);
  }
  markers_pub_->publish(marr);
}
