#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>

// Macro to warn about unset parameters
#define RCLCPP_PARAM_WARN(node, param_name, default_val)            \
  RCLCPP_WARN(node->get_logger(),                                   \
    "Param is not set: %s. Setting to default value: %s",           \
    param_name.c_str(), default_val.c_str())

class PointcloudConcatenate : public rclcpp::Node {
public:
  PointcloudConcatenate();

private:
  void handleParams();
  void update();
  void subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishPointcloud(sensor_msgs::msg::PointCloud2 &cloud);

  // Subs & pub
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in2_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in3_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in4_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string target_frame_;
  int clouds_;
  double hz_;
  // Parámetros de ICP
  bool use_icp_;
  double icp_max_corr_dist_;
  int icp_max_iterations_;


  // Buffers & flags
  sensor_msgs::msg::PointCloud2 cloud_in1_, cloud_in2_, cloud_in3_, cloud_in4_;
  bool cloud_in1_received_ = false, cloud_in2_received_ = false;
  bool cloud_in3_received_ = false, cloud_in4_received_ = false;
  bool cloud_in1_received_recent_ = false, cloud_in2_received_recent_ = false;
  bool cloud_in3_received_recent_ = false, cloud_in4_received_recent_ = false;
};
