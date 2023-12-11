#pragma once
#ifndef POINTCLOUD_CONCAT__POINTCLOUD_CONCAT_HPP_
#define POINTCLOUD_CONCAT__POINTCLOUD_CONCAT_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "pcl_ros/transforms.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pointcloud_concatenate
{

#define ROSPARAM_WARN(param_name, default_val)                               \
  std::cout << "\033[33m"                                                    \
            << "[WARN] Param is not set: " << param_name                          \
            << ". Setting to default value: " << default_val << "\033[0m\n"  \
            << std::endl

class PointCloudConcatNode : public rclcpp::Node {
private:

  void subCallbackCloudIn1(sensor_msgs::msg::PointCloud2 msg);
  void subCallbackCloudIn2(sensor_msgs::msg::PointCloud2 msg);
  void subCallbackCloudIn3(sensor_msgs::msg::PointCloud2 msg);
  void subCallbackCloudIn4(sensor_msgs::msg::PointCloud2 msg);
  void publishPointcloud(sensor_msgs::msg::PointCloud2 cloud);

  std::shared_ptr<rclcpp::Node> nh;
  std::string node_name_;
  std::string param_frame_target_;
  int param_clouds_, queue_size_;
  double param_hz_;
  rclcpp::SensorDataQoS sensor_qos;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in1, sub_cloud_in2, sub_cloud_in3, sub_cloud_in4;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::PointCloud2 cloud_in1, cloud_in2, cloud_in3, cloud_in4, cloud_out;
  bool cloud_in1_received = false;
  bool cloud_in2_received = false;
  bool cloud_in3_received = false;
  bool cloud_in4_received = false;
  bool cloud_in1_received_recent = false;
  bool cloud_in2_received_recent = false;
  bool cloud_in3_received_recent = false;
  bool cloud_in4_received_recent = false;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;

public:
  explicit PointCloudConcatNode(const rclcpp::NodeOptions & options);
  ~PointCloudConcatNode() override;

  void handleParams();
  void update();
};

}

#endif