#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ObstaclePublisher : public rclcpp::Node
{
public:
  ObstaclePublisher();

private:
  /* ── subscriber ─────────────────────────────────────────── */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // single-flag 
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              obstacle_flag_pub_;
  // new flags
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              front_clear_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              back_clear_pub_;
  // visuals
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    obstacles_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};
