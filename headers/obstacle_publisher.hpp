#ifndef OBSTACLE_DETECTOR_HPP_
#define OBSTACLE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

struct BoundingBox
{
  Eigen::Vector3f min_pt;
  Eigen::Vector3f max_pt;
  float distance = 0.0f;   ///< centre‑to‑sensor distance (m)
};

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector();                       

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             obstacle_pub_;
};

#endif  /* OBSTACLE_DETECTOR_HPP_ */
