// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/********************************************************************************************
 *  \file       semantic_slam.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef AS2_SLAM__SEMANTIC_SLAM_HPP_
#define AS2_SLAM__SEMANTIC_SLAM_HPP_

// ROS2
#include <Eigen/Dense>
#include <geometry_msgs/msg/detail/pose_with_covariance__struct.hpp>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <memory>
// ROS2 MSGS
#include <as2_msgs/msg/pose_stamped_with_id.hpp>
#include <as2_msgs/msg/pose_stamped_with_id_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <as2_core/node.hpp>
#include "as2_slam/optimizer_g2o.hpp"
#include "utils/conversions.hpp"

class SemanticSlam : public as2::Node
{
public:
  SemanticSlam(rclcpp::NodeOptions & options);
  ~SemanticSlam() {}
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void detectionsCallback(const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg);
  void arucoPoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg);
  void gatePoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg);

private:
  Eigen::Isometry3d generatePoseFromMsg(
    const as2_msgs::msg::PoseStampedWithID & _msg);

  void visualizeCleanTempGraph();
  void visualizeMainGraph();
  void visualizeTempGraph();

  visualization_msgs::msg::MarkerArray generateVizNodesMsg(std::shared_ptr<GraphG2O> & _graph);
  visualization_msgs::msg::MarkerArray generateVizEdgesMsg(std::shared_ptr<GraphG2O> & _graph);
  visualization_msgs::msg::MarkerArray generateCleanMarkersMsg();
  void updateMapOdomTransform(const std_msgs::msg::Header & _header);

  void processOdometryReceived(
    const Eigen::Isometry3d _odom_pose,
    const Eigen::MatrixXd _odom_covariance,
    const std_msgs::msg::Header & _header);
  void processArucoMsg(const as2_msgs::msg::PoseStampedWithID _msg);
  void processGateMsg(const as2_msgs::msg::PoseStampedWithID _msg);
  void processArucoMsg(
    const as2_msgs::msg::PoseStampedWithID _msg,
    const OdometryInfo _detection_odometry_info);
  void processGateMsg(
    const as2_msgs::msg::PoseStampedWithID _msg,
    const OdometryInfo _detection_odometry_info);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithIDArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr aruco_pose_sub_;
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr gate_pose_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_main_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_temp_markers_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    corrected_localization_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_path_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr initial_origin_timer_;
  std::unique_ptr<OptimizerG2O> optimizer_ptr_;  // g2o graph

  OdometryWithCovariance last_odometry_received_;
  geometry_msgs::msg::TransformStamped map_odom_transform_msg_;

  // PARAMETERS
  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_frame_;
  bool odometry_is_relative_ = false;
  std::string force_object_type_;

  // std::filesystem::path plugin_name_;
  // std::shared_ptr<pluginlib::ClassLoader<as2_state_estimator_plugin_base::StateEstimatorBase>>
  //     loader_;
  // std::shared_ptr<as2_state_estimator_plugin_base::StateEstimatorBase>
  // plugin_ptr_; std::shared_ptr<tf2_ros::TransformBroadcaster>
};

#endif  // AS2_SLAM__SEMANTIC_SLAM_HPP_
