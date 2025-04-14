#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <limits>
#include <vector>
#include <mutex>
#include <thread>
#include <algorithm>
#include <cmath>

#include "sensors_subscriber.h"

// Speed constants
static constexpr float SPEED_MAX     = 3.0f;  
static constexpr float SPEED_CAUTION = 2.0f;  
static constexpr float SPEED_STOP    = 0.0f;  

// IMU thresholds
static constexpr float ACC_LOW_THRESHOLD  = 7.0f;   
static constexpr float ACC_HIGH_THRESHOLD = 12.0f;  

// Robot states
enum class RobotState {
  NORMAL,
  CAUTION,
  STOP
};

class StateMachine {
public:
  StateMachine() : current_state_(RobotState::NORMAL) {}

  void update(float speed_limit) {
    if (speed_limit == SPEED_STOP) {
      current_state_ = RobotState::STOP;
    } else if (speed_limit == SPEED_CAUTION) {
      current_state_ = RobotState::CAUTION;
    } else {
      current_state_ = RobotState::NORMAL;
    }
  }

  std::string getStateName() const {
    switch (current_state_) {
      case RobotState::NORMAL:  return "NORMAL";
      case RobotState::CAUTION: return "CAUTION";
      case RobotState::STOP:    return "STOP";
      default:                  return "UNKNOWN";
    }
  }

private:
  RobotState current_state_;
};

// A small struct for bounding boxes
struct BoundingBox {
  int cluster_id;
  Eigen::Vector3f min_pt;
  Eigen::Vector3f max_pt;
  float distance;  // from LiDAR origin
};

class WheelchairNode : public rclcpp::Node
{
public:
  WheelchairNode()
  : Node("wheelchair_node")
  {
    // LiDAR subscription
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 
      10,
      std::bind(&WheelchairNode::cloudCallback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(get_logger(), "Subscribed to /livox/lidar.");

    // PCL Visualizer
    viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("Refined LiDAR Clusters");
    viewer_->setBackgroundColor(0, 0, 0);

    // Timer to update the PCL viewer ~20 Hz
    viewer_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&WheelchairNode::updateViewer, this)
    );

    // FSM Timer (0.5 s)
    fsm_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&WheelchairNode::updateFSM, this)
    );

    RCLCPP_INFO(get_logger(), "WheelchairNode constructor done.");
  }

  // Call this after node is constructed via std::make_shared
  void initialize()
  {
    sensor_subscriber_ = std::make_shared<SensorsSubscriber>(this->shared_from_this());
    RCLCPP_INFO(get_logger(), "SensorsSubscriber created in initialize().");
  }

private:
  // LiDAR Callback
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_in);

    if (cloud_in->empty()) {
      RCLCPP_WARN(get_logger(), "Received an empty cloud. Skipping processing.");
      return;
    }

    // Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
    {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(cloud_in);
      voxel.setLeafSize(0.1f, 0.1f, 0.1f);
      voxel.filter(*down);
    }

    // Segment ground
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2f);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    seg.setInputCloud(down);
    seg.segment(*inliers, *coeffs);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);

    if (!inliers->indices.empty()) {
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(down);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*ground);

      extract.setNegative(true);
      extract.filter(*obstacles);
    } else {
      obstacles = down; // No plane => everything is obstacles
    }

    // Remove outliers
    {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(obstacles);
      sor.setMeanK(20);
      sor.setStddevMulThresh(2.0);
      sor.filter(*obstacles);
    }

    // Cluster
    std::vector<pcl::PointIndices> cluster_indices;
    {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(obstacles);

      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(0.2f);
      ec.setMinClusterSize(30);
      ec.setSearchMethod(tree);
      ec.setInputCloud(obstacles);
      ec.extract(cluster_indices);
    }

    // Compute bounding boxes
    std::vector<BoundingBox> boxes;
    boxes.reserve(cluster_indices.size());
    int cluster_id = 0;

    for (auto & group : cluster_indices) {
      Eigen::Vector3f min_pt(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
      );
      Eigen::Vector3f max_pt(
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max()
      );

      for (int idx : group.indices) {
        auto & pt = (*obstacles)[idx];
        min_pt.x() = std::min(min_pt.x(), pt.x);
        min_pt.y() = std::min(min_pt.y(), pt.y);
        min_pt.z() = std::min(min_pt.z(), pt.z);
        max_pt.x() = std::max(max_pt.x(), pt.x);
        max_pt.y() = std::max(max_pt.y(), pt.y);
        max_pt.z() = std::max(max_pt.z(), pt.z);
      }

      Eigen::Vector3f center = (min_pt + max_pt)*0.5f;
      float distance = center.norm();

      BoundingBox bb;
      bb.cluster_id = cluster_id++;
      bb.min_pt     = min_pt;
      bb.max_pt     = max_pt;
      bb.distance   = distance;

      boxes.push_back(bb);
    }

    // Filter by bounding box size
    std::vector<BoundingBox> final_bboxes;
    for (auto & b : boxes) {
      Eigen::Vector3f size = b.max_pt - b.min_pt;
      float dx = size.x(), dy = size.y(), dz = size.z();
      bool pass_max = (dx < 3.0f && dy < 3.0f && dz < 3.0f);
      bool pass_min = (dx > 0.1f || dy > 0.1f || dz > 0.1f);
      if (pass_max && pass_min) {
        final_bboxes.push_back(b);
      }
    }

    // Store for FSM & viewer
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      ground_cloud_    = ground;
      obstacles_cloud_ = obstacles;
      cluster_indices_ = cluster_indices;
      final_bboxes_    = final_bboxes;
    }

    // Debug prints
    RCLCPP_INFO(get_logger(),
      "LiDAR => Down:%zu, Ground:%zu, Obs:%zu, Clusters:%zu, BBoxes:%zu",
      down->size(),
      ground->size(),
      obstacles->size(),
      cluster_indices.size(),
      final_bboxes.size()
    );
    for (auto & b : final_bboxes) {
      RCLCPP_INFO(get_logger(),
        "  -> Cluster %d: distance= %.2f m", b.cluster_id, b.distance);
    }
  }

  // PCL Viewer ~20 Hz
  void updateViewer()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground, obstacles;
    std::vector<pcl::PointIndices> clusters;
    std::vector<BoundingBox> bboxes;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      ground    = ground_cloud_;
      obstacles = obstacles_cloud_;
      clusters  = cluster_indices_;
      bboxes    = final_bboxes_;
    }

    if (!ground || !obstacles) {
      return; 
    }

    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();

    // Ground in green
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(ground, 0, 255, 0);
      viewer_->addPointCloud<pcl::PointXYZ>(ground, green, "ground_cloud");
    }

    // Obstacles in white
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(obstacles, 255, 255, 255);
      viewer_->addPointCloud<pcl::PointXYZ>(obstacles, white, "obstacles_cloud");
    }

    // Each cluster in random color + bounding boxes
    int i = 0;
    for (auto & group : clusters) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr ccloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (int idx : group.indices) {
        ccloud->push_back((*obstacles)[idx]);
      }

      uint8_t r = std::rand() % 256;
      uint8_t g = std::rand() % 256;
      uint8_t b = std::rand() % 256;
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(ccloud, r, g, b);

      std::string cname = "cluster_" + std::to_string(i);
      viewer_->addPointCloud<pcl::PointXYZ>(ccloud, color, cname);

      // Matching bounding box
      for (auto & box : bboxes) {
        if (box.cluster_id == i) {
          viewer_->addCube(
            box.min_pt.x(), box.max_pt.x(),
            box.min_pt.y(), box.max_pt.y(),
            box.min_pt.z(), box.max_pt.z(),
            static_cast<double>(r)/255.0,
            static_cast<double>(g)/255.0,
            static_cast<double>(b)/255.0,
            "bbox_" + std::to_string(i)
          );
          viewer_->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
            "bbox_" + std::to_string(i)
          );

          // Distance text
          Eigen::Vector3f c = (box.min_pt + box.max_pt)*0.5f;
          char dist[64];
          std::snprintf(dist, 64, "Dist: %.2f m", box.distance);
          viewer_->addText3D<pcl::PointXYZ>(
            dist,
            pcl::PointXYZ(c.x(), c.y(), box.max_pt.z() + 0.3f),
            0.2f,
            1.0, 1.0, 1.0,
            "dist_text_" + std::to_string(i)
          );
          break;
        }
      }

      i++;
    }

    viewer_->spinOnce(10);
  }

  // Timer: FSM logic ~0.5 s
  void updateFSM()
  {
    // 1) Copy bounding boxes
    std::vector<BoundingBox> bboxes_copy;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      bboxes_copy = final_bboxes_;
    }

    // 2) Retrieve sensor data
    auto sensor_data = sensor_subscriber_->get_latest_sensor_data();

    //  Print the raw sensor data before final speed calc
    RCLCPP_INFO(get_logger(),
      "[SENSORS] Ultras=%.2f, IMU=(%.2f, %.2f, %.2f), PIR=(%s,%s,%s,%s)",
      sensor_data.ultrasonic_front_0,
      sensor_data.linear_acceleration_x,
      sensor_data.linear_acceleration_y,
      sensor_data.linear_acceleration_z,
      (sensor_data.pir_front ? "true" : "false"),
      (sensor_data.pir_back  ? "true" : "false"),
      (sensor_data.pir_left  ? "true" : "false"),
      (sensor_data.pir_right ? "true" : "false")
    );

    // Ultrasonic => speed limit
    float ultrasonic = sensor_data.ultrasonic_front_0;
    float speed_ultr = (ultrasonic >= 1.0f) ? SPEED_MAX :
                       (ultrasonic >= 0.5f) ? SPEED_CAUTION : SPEED_STOP;

    // IMU => speed limit
    float ax = sensor_data.linear_acceleration_x;
    float ay = sensor_data.linear_acceleration_y;
    float az = sensor_data.linear_acceleration_z;
    float accMag = std::sqrt(ax*ax + ay*ay + az*az);

    float speed_imu = SPEED_MAX;
    if ((accMag < ACC_LOW_THRESHOLD) || (accMag > ACC_HIGH_THRESHOLD)) {
      speed_imu = SPEED_STOP;
    }

    float final_speed = std::min(speed_ultr, speed_imu);

    // PIR => any triggered => STOP
    bool pir_triggered = (sensor_data.pir_front ||
                          sensor_data.pir_back  ||
                          sensor_data.pir_left  ||
                          sensor_data.pir_right);
    if (pir_triggered) {
      final_speed = std::min(final_speed, SPEED_STOP);
    }

    // LiDAR => if cluster <1.0 m => STOP
    for (auto & box : bboxes_copy) {
      if (box.distance < 1.0f) {
        final_speed = std::min(final_speed, SPEED_STOP);
      }
    }

    // Update FSM
    fsm_.update(final_speed);

    // Print final speed/state
    RCLCPP_INFO(get_logger(),
      "[FSM] Ultras=%.2f, IMU=%.2f, PIR=%s, Clusters=%zu => Speed=%.2f, State=%s",
      ultrasonic, 
      accMag,
      (pir_triggered ? "true" : "false"),
      bboxes_copy.size(),
      final_speed,
      fsm_.getStateName().c_str()
    );
  }

  //  Member Variables
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr viewer_timer_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;

  // LiDAR data (protected by mutex)
  std::mutex data_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_{nullptr};
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud_{nullptr};
  std::vector<pcl::PointIndices> cluster_indices_;
  std::vector<BoundingBox> final_bboxes_;

  // PCL Visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer_;

  // Sensors + FSM
  std::shared_ptr<SensorsSubscriber> sensor_subscriber_;
  StateMachine fsm_;
};

// Standard ROS 2 main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WheelchairNode>();
  node->initialize(); // Now safe to call shared_from_this() inside the node

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
