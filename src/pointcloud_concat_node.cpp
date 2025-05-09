#include "pointcloud_concatenate_ros2/pointcloud_concat_node.hpp"

namespace pointcloud_concatenate
{

PointCloudConcatNode::PointCloudConcatNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_concatenate", options), reliable_qos(rclcpp::QoS(10).reliable())
{
  using std::placeholders::_1;
  handleParams();
  tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

  sub_cloud_in1 = this->create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in1", reliable_qos, std::bind(&PointCloudConcatNode::subCallbackCloudIn1, this, _1));
  sub_cloud_in2 = this->create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in2", reliable_qos, std::bind(&PointCloudConcatNode::subCallbackCloudIn2, this, _1));
  sub_cloud_in3 = this->create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in3", reliable_qos, std::bind(&PointCloudConcatNode::subCallbackCloudIn3, this, _1));
  sub_cloud_in4 = this->create_subscription<sensor_msgs::msg::PointCloud2>("cloud_in4", reliable_qos, std::bind(&PointCloudConcatNode::subCallbackCloudIn4, this, _1));
  pub_cloud_out = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", reliable_qos);

  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/(this->param_hz_))), std::bind(&PointCloudConcatNode::update, this));
}

PointCloudConcatNode::~PointCloudConcatNode()
{
  RCLCPP_INFO(this->get_logger(), "Destructing PointcloudConcatenate...");
}

void PointCloudConcatNode::subCallbackCloudIn1(sensor_msgs::msg::PointCloud2 msg)
{
  cloud_in1 = msg;
  cloud_in1_received = true;
  cloud_in1_received_recent = true;
}

void PointCloudConcatNode::subCallbackCloudIn2(sensor_msgs::msg::PointCloud2 msg)
{
  cloud_in2 = msg;
  cloud_in2_received = true;
  cloud_in2_received_recent = true;
}

void PointCloudConcatNode::subCallbackCloudIn3(sensor_msgs::msg::PointCloud2 msg)
{
  cloud_in3 = msg;
  cloud_in3_received = true;
  cloud_in3_received_recent = true;
}

void PointCloudConcatNode::subCallbackCloudIn4(sensor_msgs::msg::PointCloud2 msg)
{
  cloud_in4 = msg;
  cloud_in4_received = true;
  cloud_in4_received_recent = true;
}

void PointCloudConcatNode::handleParams()
{
  RCLCPP_INFO(this->get_logger(), "Loading parameters...");
  std::string param_name;

  std::string parse_str;
  param_name = "/target_frame";
  declare_parameter(param_name, parse_str);
  parse_str = get_parameter(param_name).as_string().c_str();
  if (!(parse_str.length() > 0)) {
      param_frame_target_ = "base_link";
      ROSPARAM_WARN(param_name, param_frame_target_);
  }
  param_frame_target_ = parse_str;

  param_name = "/clouds";
  declare_parameter(param_name, param_clouds_);
  param_clouds_ = get_parameter(param_name).as_int();
  if (!param_clouds_) {
      param_clouds_ = 2;
      ROSPARAM_WARN(param_name, param_clouds_);
  }
  else if(param_clouds_>4){
      param_clouds_ = 2;
      RCLCPP_WARN(this->get_logger(), "Max only 4 inputs are supported yet. Already reset to 2.");
  }

  // Frequency to update/publish
  param_name = "/hz";
  declare_parameter(param_name, param_hz_);
  param_hz_ = get_parameter(param_name).as_double();
  if (!param_hz_) {
      param_hz_ = 10.0;
      ROSPARAM_WARN(param_name, param_hz_);
  }

  RCLCPP_INFO(this->get_logger(), "Parameters loaded.");
}

void PointCloudConcatNode::update() {
  if (pub_cloud_out->get_subscription_count() > 0 && param_clouds_ >= 1) {
    sensor_msgs::msg::PointCloud2 cloud_to_concat;
    cloud_out = cloud_to_concat;
    bool success = true;

    if ((!cloud_in1_received) && (!cloud_in2_received) && (!cloud_in3_received) && (!cloud_in4_received)) {
      RCLCPP_WARN_SKIPFIRST(this->get_logger(),"No pointclouds received yet. Waiting 1 second...");
      rclcpp::sleep_for(std::chrono::seconds(1));
      return;
    }
    if (param_clouds_ >= 1 && success && cloud_in1_received) {
      if (!cloud_in1_received_recent) {
        RCLCPP_WARN_SKIPFIRST(this->get_logger(),"Cloud 1 was not received since last update, reusing last received message...");
      }
      cloud_in1_received_recent = false;

      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in1, cloud_out, *tfBuffer);
      if (!success) {
        RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Transforming cloud 1 from %s to %s failed!", cloud_in1.header.frame_id.c_str(), param_frame_target_.c_str());
      }
    }

    if (param_clouds_ >= 2 && success && cloud_in2_received) {
      if (!cloud_in2_received_recent) {
        RCLCPP_WARN_SKIPFIRST(this->get_logger(),"Cloud 2 was not received since last update, reusing last received message...");
      }
      cloud_in2_received_recent = false;

      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in2, cloud_to_concat, *tfBuffer);
      if (!success) {
        RCLCPP_WARN_SKIPFIRST(this->get_logger(),"Transforming cloud 2 from %s to %s failed!", cloud_in2.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    if (param_clouds_ >= 3 && success && cloud_in3_received) {
      if (!cloud_in3_received_recent) {
        RCLCPP_WARN_SKIPFIRST(this->get_logger(),"Cloud 3 was not received since last update, reusing last received message...");
      }
      cloud_in3_received_recent = false;

      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in3, cloud_to_concat, *tfBuffer);
      if (!success) {
        RCLCPP_WARN_SKIPFIRST(this->get_logger(),"Transforming cloud 3 from %s to %s failed!", cloud_in3.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }

    if (param_clouds_ >= 4 && success && cloud_in4_received) {
      if (!cloud_in4_received_recent) {
        RCLCPP_WARN_SKIPFIRST(this->get_logger(),"Cloud 4 was not received since last update, reusing last received message...");
      }
      cloud_in4_received_recent = false;

      success = pcl_ros::transformPointCloud(param_frame_target_, cloud_in4, cloud_to_concat, *tfBuffer);
      if (!success) {
        RCLCPP_WARN_SKIPFIRST(this->get_logger(),"Transforming cloud 4 from %s to %s failed!", cloud_in4.header.frame_id.c_str(), param_frame_target_.c_str());
      }

      if (success) {
        pcl::concatenatePointCloud(cloud_out, cloud_to_concat, cloud_out);
      }
    }
    if (success) {
      publishPointcloud(cloud_out);
    }
  }
}

void PointCloudConcatNode::publishPointcloud(sensor_msgs::msg::PointCloud2 cloud)
{
    cloud.header.stamp = rclcpp::Clock().now();
    pub_cloud_out->publish(cloud);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_concatenate::PointCloudConcatNode)
