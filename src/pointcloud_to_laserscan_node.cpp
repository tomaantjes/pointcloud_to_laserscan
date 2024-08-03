/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.5);
  lidar_frame_level_ = this->declare_parameter("lidar_frame_level", "lidar_frame_level");
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  // std::string pose_topic_ = this->declare_parameter("pose_topic", "/visual_slam/tracking/vo_pose");
  std::string pose_topic_ = this->declare_parameter("pose_topic", "/mavros/local_position/pose");
  rclcpp::QoS qos_R(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_R.reliable();
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos_R);

  rclcpp::QoS qos_best_effort(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_best_effort.best_effort();
  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, qos_best_effort, std::bind(&PointCloudToLaserScanNode::poseCallback, this, std::placeholders::_1));
  
  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}


void PointCloudToLaserScanNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
  tf2::Quaternion q(
    pose_msg->pose.orientation.x,
    pose_msg->pose.orientation.y,
    pose_msg->pose.orientation.z,
    pose_msg->pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(latest_roll_, latest_pitch_, latest_yaw_);
  // RCLCPP_INFO(this->get_logger(), "ROLL: %f PITCH: %f", latest_roll_, latest_pitch_);
  // std::cout << latest_roll_ << "ROLL " << latest_pitch_ << "PITCH";
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    // scan_msg->header.frame_id = target_frame_;
    scan_msg->header.frame_id = lidar_frame_level_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  // if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
  //   RCLCPP_INFO(this->get_logger(), "Transforming point cloud from frame %s to frame %s",
  //     cloud_msg->header.frame_id.c_str(), scan_msg->header.frame_id.c_str());
  //   try {
  //     auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  //     tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
  //     cloud_msg = cloud;
  //   } catch (tf2::TransformException & ex) {
  //     RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
  //     return;
  //   }
  // }
  try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, lidar_frame_level_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }

  // Create a transformation matrix from the roll and pitch
  tf2::Quaternion q;
  q.setRPY(latest_roll_, latest_pitch_, 0.0);  // Only roll and pitch, no yaw
  tf2::Matrix3x3 mat(q);

  tf2::Vector3 rotated_point;
  
  int n_inf = 0;
  int n = 0;
  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    
    n++;
    // Check if iter_x, iter_y, iter_z are infinite and if so set to range_max
    if (std::isinf(*iter_x) || std::isinf(*iter_y) || std::isinf(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for infinite in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      n_inf++;
      continue;
    }
  
    // RCLCPP_INFO(this->get_logger(), "z_before: %f z_after: %f", *iter_z, rotated_point.getZ());
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }    
    
    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (fabs(*iter_z) > 0.3 * std::min(range / 6, 0.33)) { 
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }
    
    // overwrite range at laserscan ray if new range is smaller
    // RCLCPP_INFO(this->get_logger(), "ANGLE: %f x: %f y: %f z: %f x_orig: %f, y_orig: %f, z_orig: %f" , angle * 180 / M_PI, rotated_point.getX(), rotated_point.getY(), rotated_point.getZ(), *iter_x, *iter_y, *iter_z);
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }
  // RCLCPP_INFO(this->get_logger(), "Number of points: %d, Number of inf: %d", n, n_inf);
  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
