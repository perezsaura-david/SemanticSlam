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

/**
 * @file semantic_slam.cpp
 *
 * SemanticSlam class implementation for AeroStack2
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "as2_slam/semantic_slam.hpp"
#include <Eigen/src/Geometry/Transform.h>
#include <g2o/core/optimizable_graph.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <memory>
#include <string>
#include <vector>

#include "as2_slam/graph_node_types.hpp"
#include "utils/conversions.hpp"
#include "as2_slam/object_detection_types.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

// #include <filesystem>
// #include <pluginlib/class_loader.hpp>
// #include "plugin_base.hpp"

SemanticSlam::SemanticSlam()
: as2::Node("semantic_slam")
{
  std::string default_odom_topic = "drone/sensor_measurements/odom";
  std::string default_corrected_localization_topic = "drone/slam/corrected_localization";
  std::string default_corrected_path_topic = "drone/slam/corrected_path";
  std::string default_aruco_pose_topic = "drone/detections/aruco";
  std::string default_gate_pose_topic = "drone/detections/gate";
  std::string default_detections_topic = "drone/detections";
  std::string default_viz_main_markers_topic = "slam_viz/main";
  std::string default_viz_temp_markers_topic = "slam_viz/temp";
  std::string default_map_frame = "drone/map";
  std::string default_odom_frame = "drone/odom";
  std::string default_robot_frame = "drone/base_link";

  rclcpp::QoS reliable_qos = rclcpp::QoS(10);
  rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

  // PARAMETERS
  std::string odom_topic = this->declare_parameter("odometry_topic", default_odom_topic);
  std::string corrected_localization_topic = this->declare_parameter(
    "localization_topic", default_corrected_localization_topic);
  std::string aruco_pose_topic =
    this->declare_parameter("aruco_pose_topic", default_aruco_pose_topic);
  std::string gate_pose_topic =
    this->declare_parameter("gate_pose_topic", default_gate_pose_topic);
  std::string detections_topic =
    this->declare_parameter("detections_topic", default_detections_topic);
  map_frame_ =
    this->declare_parameter<std::string>("map_frame", default_map_frame);
  odom_frame_ =
    this->declare_parameter<std::string>("odom_frame", default_odom_frame);
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", default_robot_frame);
  // VISUALIZATION
  std::string viz_main_markers_topic =
    this->declare_parameter("viz_main_markers_topic", default_viz_main_markers_topic);
  std::string viz_temp_markers_topic =
    this->declare_parameter("viz_temp_markers_topic", default_viz_temp_markers_topic);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, sensor_qos, std::bind(&SemanticSlam::odomCallback, this, std::placeholders::_1));
  aruco_pose_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
    aruco_pose_topic, sensor_qos,
    std::bind(&SemanticSlam::arucoPoseCallback, this, std::placeholders::_1));
  gate_pose_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
    gate_pose_topic, sensor_qos,
    std::bind(&SemanticSlam::gatePoseCallback, this, std::placeholders::_1));
  detections_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithIDArray>(
    detections_topic, sensor_qos,
    std::bind(&SemanticSlam::detectionsCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  corrected_localization_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    corrected_localization_topic, reliable_qos);
  corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    default_corrected_path_topic, reliable_qos);
  viz_main_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    viz_main_markers_topic, reliable_qos);
  viz_temp_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    viz_temp_markers_topic, reliable_qos);

  optimizer_ptr_ = std::make_unique<OptimizerG2O>();

  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  updateMapOdomTransform(header);
}

void SemanticSlam::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  Eigen::Isometry3d odom_pose = convertToIsometry3d(msg->pose.pose);
  Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> odom_covariance(
    msg->pose.covariance.data());

  if (odom_pose.translation().isZero()) {return;}
  // Eigen::Matrix<double, 6, 6> odom_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.1;

  OdometryWithCovariance odometry_received_;
  odometry_received_.odometry = odom_pose;
  odometry_received_.covariance = odom_covariance;

  // TODO(dps): Define how to use this
  // msg->header.stamp;
  bool new_node_added = optimizer_ptr_->handleNewOdom(odometry_received_);

  last_odometry_received_ = odometry_received_;

  if (new_node_added) {
    visualizeMainGraph();
    visualizeCleanTempGraph();
    updateMapOdomTransform(msg->header);
  }

  map_odom_transform_msg_.header.stamp = msg->header.stamp;
  tf_broadcaster_->sendTransform(map_odom_transform_msg_);

  // Get map_odom_transform from optimizer and publish corrected localization
  Eigen::Isometry3d map_odom_transform = optimizer_ptr_->getMapOdomTransform();
  Eigen::Isometry3d corrected_odometry_pose = map_odom_transform * odom_pose;
  geometry_msgs::msg::PoseWithCovarianceStamped corrected_localization_msg;
  geometry_msgs::msg::PoseStamped pose_stamped_msg;
  pose_stamped_msg.pose = convertToGeometryMsgPose(corrected_odometry_pose);
  corrected_localization_msg.pose.pose = pose_stamped_msg.pose;
  corrected_localization_msg.header.stamp = msg->header.stamp;
  corrected_localization_msg.header.frame_id = map_frame_;
  corrected_localization_pub_->publish(corrected_localization_msg);

  // Publish corrected Path
  static nav_msgs::msg::Path corrected_path_msg;
  corrected_path_msg.header.stamp = msg->header.stamp;
  corrected_path_msg.header.frame_id = map_frame_;
  corrected_path_msg.poses.emplace_back(pose_stamped_msg);
  corrected_path_pub_->publish(corrected_path_msg);
}

void SemanticSlam::arucoPoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg)
{
  processArucoMsg(*msg);
}

void SemanticSlam::gatePoseCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr msg)
{
  processGateMsg(*msg);
}

void SemanticSlam::processGateMsg(const as2_msgs::msg::PoseStampedWithID _msg)
{
  std::string gate_id = _msg.id;
  Eigen::Vector3d gate_position = generatePoseFromMsg(_msg).translation();
  Eigen::Matrix<double, 3, 3> gate_covariance = Eigen::MatrixXd::Identity(3, 3) * 0.001;

  bool detections_are_absolute = false;

  GateDetection * gate(new GateDetection(
      gate_id, gate_position, gate_covariance,
      detections_are_absolute));

  optimizer_ptr_->handleNewObjectDetection(gate, last_odometry_received_);
  visualizeTempGraph();
}

void SemanticSlam::detectionsCallback(
  const as2_msgs::msg::PoseStampedWithIDArray::SharedPtr msg)
{
  std::string object_type;
  // object_type = msg->type;
  object_type = "aruco";
  for (auto & detection : msg->poses) {
    if (object_type == "aruco") {
      processArucoMsg(detection);
    } else if (object_type == "gate") {
      processGateMsg(detection);
    }
  }
}

void SemanticSlam::processArucoMsg(const as2_msgs::msg::PoseStampedWithID _msg)
{
  std::string aruco_id = _msg.id;
  // TODO(dps): Define how to use this
  // msg->pose.header.stamp;
  Eigen::Isometry3d aruco_pose = generatePoseFromMsg(_msg);
  Eigen::Matrix<double, 6, 6> aruco_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.001;
  // aruco_covariance(0) = 0.001;
  // aruco_covariance(7) = 0.001;
  // aruco_covariance(14) = 0.001;
  // aruco_covariance(21) = 0.001;

  bool detections_are_absolute = false;

  ArucoDetection * aruco(new ArucoDetection(
      aruco_id, aruco_pose, aruco_covariance,
      detections_are_absolute));

  optimizer_ptr_->handleNewObjectDetection(aruco, last_odometry_received_);
  visualizeTempGraph();
}


void SemanticSlam::updateMapOdomTransform(const std_msgs::msg::Header & _header)
{
  map_odom_transform_msg_ = convertToTransformStamped(
    optimizer_ptr_->getMapOdomTransform(), map_frame_, odom_frame_, _header.stamp);
}

// Eigen::Isometry3d SemanticSlam::generatePoseFromMsg(
//   const std::shared_ptr<as2_msgs::msg::PoseStampedWithID> & _msg)
Eigen::Isometry3d SemanticSlam::generatePoseFromMsg(
  const as2_msgs::msg::PoseStampedWithID & _msg)
{
  Eigen::Isometry3d pose;
  std::string ref_frame = _msg.pose.header.frame_id;
  // auto target_ts        = tf2::TimePointZero;
  auto target_ts = _msg.pose.header.stamp;
  std::chrono::nanoseconds tf_timeout =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0));
  auto tf_names = tf_buffer_->getAllFrameNames();

  // FIXME: We need this in simulation because of the current optical link implementation in
  // gazebo
  for (auto tf_name : tf_names) {
    if (tf_name.find(ref_frame) < tf_name.length()) {
      ref_frame = tf_name;
      // WARN("Ref frame changed to: " << ref_frame);
    }
  }

  // CHANGED REFERENCE_FRAME TO ROBOT_FRAME
  if (ref_frame != robot_frame_) {
    geometry_msgs::msg::TransformStamped ref_frame_transform;
    try {
      // WARN("Transform detection from " << ref_frame << " to " << robot_frame_);
      ref_frame_transform =
        tf_buffer_->lookupTransform(robot_frame_, ref_frame, target_ts, tf_timeout);
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(_msg.pose, transformed_pose, ref_frame_transform);
      pose = convertToIsometry3d(transformed_pose.pose);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s", ref_frame.c_str(),
        robot_frame_.c_str(), ex.what());
    }
  } else {
    pose = convertToIsometry3d(_msg.pose.pose);
  }
  return pose;
}

void SemanticSlam::visualizeMainGraph()
{
  visualization_msgs::msg::MarkerArray viz_odom_nodes_msg =
    generateVizNodesMsg(optimizer_ptr_->main_graph);
  viz_main_markers_pub_->publish(viz_odom_nodes_msg);
  visualization_msgs::msg::MarkerArray viz_edges_msg =
    generateVizEdgesMsg(optimizer_ptr_->main_graph);
  viz_main_markers_pub_->publish(viz_edges_msg);
}

void SemanticSlam::visualizeTempGraph()
{
  visualization_msgs::msg::MarkerArray viz_odom_nodes_msg =
    generateVizNodesMsg(optimizer_ptr_->temp_graph);
  viz_temp_markers_pub_->publish(viz_odom_nodes_msg);
  visualization_msgs::msg::MarkerArray viz_edges_msg =
    generateVizEdgesMsg(optimizer_ptr_->temp_graph);
  viz_temp_markers_pub_->publish(viz_edges_msg);
}

void SemanticSlam::visualizeCleanTempGraph()
{
  visualization_msgs::msg::MarkerArray viz_clean_markers_msg = generateCleanMarkersMsg();
  viz_temp_markers_pub_->publish(viz_clean_markers_msg);
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateVizNodesMsg(
  std::shared_ptr<GraphG2O> & _graph)
{
  bool main = false;
  if (_graph->getName() == "Main Graph") {
    main = true;
  }
  visualization_msgs::msg::MarkerArray viz_markers_msg;
  std::vector<GraphNode *> graph_nodes = _graph->getNodes();
  for (auto & node : graph_nodes) {
    visualization_msgs::msg::Marker viz_marker_msg = node->getVizMarker(main);
    viz_marker_msg.header.frame_id = map_frame_;
    viz_markers_msg.markers.emplace_back(viz_marker_msg);
    // visualization_msgs::msg::Marker viz_cov_marker_msg = node->getVizCovMarker();
    // viz_markers_msg.markers.emplace_back(viz_marker_msg);
  }
  return viz_markers_msg;
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateVizEdgesMsg(
  std::shared_ptr<GraphG2O> & _graph)
{
  bool main = false;
  if (_graph->getName() == "Main Graph") {
    main = true;
  }
  visualization_msgs::msg::MarkerArray viz_markers_msg;
  std::vector<GraphEdge *> graph_edges = _graph->getEdges();
  for (auto & edge : graph_edges) {
    visualization_msgs::msg::Marker viz_marker_msg = edge->getVizMarker(main);
    viz_marker_msg.header.frame_id = map_frame_;
    viz_markers_msg.markers.emplace_back(viz_marker_msg);
  }
  return viz_markers_msg;
}

visualization_msgs::msg::MarkerArray SemanticSlam::generateCleanMarkersMsg()
{
  visualization_msgs::msg::MarkerArray markers_msg;
  visualization_msgs::msg::Marker marker_msg;
  // marker_msg.ns     = _namespace;
  marker_msg.id = 0;
  marker_msg.action = visualization_msgs::msg::Marker::DELETEALL;
  markers_msg.markers.emplace_back(marker_msg);
  return markers_msg;
}
