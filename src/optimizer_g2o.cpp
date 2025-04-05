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
 * @file optimizer_g2o.cpp
 *
 * OptimizerG2O class implementation for AeroStack2
 *
 * @author David Pérez Saura
 *         Miguel Fernández Cortizas
 */

#include "as2_slam/optimizer_g2o.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include "as2_slam/graph_g2o.hpp"
#include "as2_slam/graph_node_types.hpp"
#include "as2_slam/object_detection_types.hpp"
#include "utils/conversions.hpp"
#include "utils/general_utils.hpp"
#include "utils/debug_utils.hpp"

OptimizerG2O::OptimizerG2O()
{
  FLAG("STARTING SEMANTIC SLAM");

  main_graph = std::make_shared<GraphG2O>("Main Graph");
  temp_graph = std::make_shared<GraphG2O>("Temp Graph");

  if (odometry_is_relative_) {
    WARN("Relative odometry");
  } else {
    WARN("Absolute odometry");
  }

  main_graph->initGraph();
  map_odom_tranform_ = Eigen::Isometry3d::Identity();
  last_odometry_added_.odometry = Eigen::Isometry3d::Identity();
  last_odometry_added_.covariance = Eigen::MatrixXd::Zero(6, 6);
  init_main_graph_ = true;
}

bool OptimizerG2O::generateOdometryInfo(
  const OdometryWithCovariance & _new_odometry,
  const OdometryWithCovariance & _last_odometry_added,
  OdometryInfo & _odometry_info)
{
  if (_new_odometry.covariance.isZero()) {
    WARN("Received covariance matrix is zero");
    return false;
  }

  if (odometry_is_relative_) {
    // TODO(dps): RELATIVE ODOMETRY
    // relative_pose = odom_pose;
    WARN("RELATIVE ODOMETRY NOT IMPLEMENTED");
    return false;
  } else {
    // ABSOLUTE ODOMETRY
    _odometry_info.odom_ref = _new_odometry.odometry;
    _odometry_info.increment = _last_odometry_added.odometry.inverse() * _odometry_info.odom_ref;
    _odometry_info.covariance_matrix = _new_odometry.covariance;
    // _odometry_info.covariance_matrix = _new_odometry.covariance - _last_odometry_added.covariance;
  }
  _odometry_info.map_ref = initial_earth_to_map_transform_ * _odometry_info.odom_ref;
  // _odometry_info.map_ref = earth_map_transform_ * _odometry_info.odom_ref;
  // _odometry_info.map_ref = _odometry_info.odom_ref;

  if (_odometry_info.covariance_matrix.isZero()) {
    WARN("Generated odometry covariance matrix is zero");
    return false;
  }

  return true;
}

bool OptimizerG2O::handleNewOdom(
  const OdometryWithCovariance & _new_odometry)
{
  OdometryInfo new_odometry_info;
  if (!generateOdometryInfo(
      _new_odometry, last_odometry_added_,
      new_odometry_info))
  {
    return false;
  }

  if (!checkAddingConditions(new_odometry_info, main_graph_odometry_distance_threshold_)) {
    return false;
  }
  last_odometry_added_.odometry = new_odometry_info.odom_ref;
  last_odometry_added_.covariance = _new_odometry.covariance;

  // DEBUG(new_odometry_info.covariance_matrix);

  // FLAG("ADDING NEW ODOMETRY TO MAIN GRAPH");
  if (std::isnan(new_odometry_info.odom_ref.translation().x())) {
    WARN("Odometry generated is NaN");
    return false;
  }

  main_graph->addNewKeyframe(
    new_odometry_info.map_ref, new_odometry_info.increment,
    new_odometry_info.covariance_matrix);

  if (temp_graph_generated_) {
    temp_graph->optimizeGraph();
    // FLAG("ADD TEMP GRAPH DETECTIONS TO MAIN GRAPH");
    for (auto object : temp_graph->getObjectNodes()) {
      // TODO(dps): Get all the nodes related and create new edges
      ObjectDetection * object_detection;
      ArucoNode * aruco_node = dynamic_cast<ArucoNode *>(object.second);
      if (aruco_node) {
        Eigen::MatrixXd cov_matrix = temp_graph->computeNodeCovariance(aruco_node);
        if (cov_matrix.size() == 0) {
          WARN("Matrix is empty! Using default matrix");
          // cov_matrix = main_graph_object_covariance;
          continue;
        }

        object_detection = new ArucoDetection(
          object.first,
          aruco_node->getPose(), cov_matrix, true);
      }
      GateNode * gate_node = dynamic_cast<GateNode *>(object.second);
      if (gate_node) {
        Eigen::MatrixXd cov_matrix = temp_graph->computeNodeCovariance(gate_node);
        if (cov_matrix.size() == 0) {
          WARN("Matrix is empty! Using default matrix");
          // cov_matrix = main_graph_object_covariance;
          continue;
        }

        object_detection = new GateDetection(
          object.first,
          gate_node->getPosition(), cov_matrix, true);
      }

      if (!object_detection->prepareMeasurements(new_odometry_info)) {
        ERROR("Prepare detection ERROR");
        continue;
      }
      // FIXME(dps): get object edge covariance
      main_graph->addNewObjectDetection(object_detection);
    }

    auto sharing = temp_graph.use_count();
    if (sharing > 1) {
      DEBUG("Temp graph Shared: " << sharing);
    }
    temp_graph.reset();
    temp_graph = std::make_shared<GraphG2O>("Temp Graph");
    temp_graph_generated_ = false;
  }

  // TODO(dps): Choose when to optimize: either every time a new keyframe is added, or every certain
  // period of time
  main_graph->optimizeGraph();
  if (generate_odom_map_transform_) {
    updateOdomMapTransform();
  }
  // debugGraphVertices(main_graph);

  return true;
}

bool OptimizerG2O::checkAddingConditions(
  const OdometryInfo & _odometry, const double _distance_threshold)
{
  // TODO(dps): check time from the last odometry received
  // FIXME(dps): get rotation distance
  double translation_distance_from_last_node = _odometry.increment.translation().norm();
  if (translation_distance_from_last_node < _distance_threshold) {
    if (init_main_graph_) {
      init_main_graph_ = false;
      return true;
    }
    return false;
  }
  return true;
}

bool OptimizerG2O::checkAddingNewDetection(
  const OdometryWithCovariance & _detection_odometry,
  OdometryInfo & _detection_odometry_info)
{
  if (!temp_graph_generated_) {
    last_detection_odometry_added_ = last_odometry_added_;
  }

  if (!generateOdometryInfo(
    _detection_odometry, last_detection_odometry_added_,
    _detection_odometry_info)) {
    return false;
  }

  if (!temp_graph_generated_) {
    temp_graph->initGraph(_detection_odometry_info.map_ref);
    // temp_graph->initGraph(main_graph->getLastOdomNode()->getPose());
    temp_graph_generated_ = true;
    // main_graph_object_covariance = _object->getCovarianceMatrix();  // FIXME(dps): remove this
  } else {
    if (!checkAddingConditions(_detection_odometry_info, tmep_graph_odometry_distance_threshold_)) {
      return false;
    }
    temp_graph->addNewKeyframe(
      _detection_odometry_info.map_ref, _detection_odometry_info.increment,
      _detection_odometry_info.covariance_matrix);
  }

  last_detection_odometry_added_.odometry = _detection_odometry_info.odom_ref;
  last_detection_odometry_added_.covariance = _detection_odometry.covariance;

  return true;
}

// TODO(dps): return bool?
void OptimizerG2O::handleNewObjectDetection(
  ObjectDetection * _object,
  const OdometryInfo & _detection_odometry_info)
{
  if (!_object->prepareMeasurements(_detection_odometry_info)) {
    ERROR("Prepare detection ERROR");
    return;
  }

  temp_graph->addNewObjectDetection(_object);
  // debugGraphVertices(temp_graph);
  // temp_graph->optimizeGraph();
  // debugGraphVertices(temp_graph);
}

void OptimizerG2O::setParameters(const OptimizerG2OParameters & _params)
{
  main_graph_odometry_distance_threshold_ = _params.main_graph_odometry_distance_threshold;
  main_graph_odometry_orientation_threshold_ = _params.main_graph_odometry_orientation_threshold;
  tmep_graph_odometry_distance_threshold_ = _params.temp_graph_odometry_distance_threshold;
  temp_graph_odometry_orientation_threshold_ = _params.temp_graph_odometry_orientation_threshold;
  map_odom_security_threshold_ = _params.map_odom_security_threshold;
  odometry_is_relative_ = _params.odometry_is_relative;
  generate_odom_map_transform_ = _params.generate_odom_map_transform;
  fixed_objects_ = _params.fixed_objects;
  initial_earth_to_map_transform_ = _params.earth_to_map_transform;
  earth_map_transform_ = initial_earth_to_map_transform_;

  Eigen::MatrixXd earth_to_map_covariance_ = Eigen::MatrixXd::Identity(6, 6) * 0.0001;
  earth_to_map_covariance_(5, 5) = 0.1;
  // odometry_received_.covariance(4, 4) *= 10e4;
  // odometry_received_.covariance(3, 3) *= 10e4;
  //
  main_graph->addNewKeyframe(initial_earth_to_map_transform_, 
    initial_earth_to_map_transform_, 
    earth_to_map_covariance_);

  main_graph->setMapNode(main_graph->getLastOdomNode());

  // last_odometry_added_.odometry = earth_to_map_transform_;
  // last_odometry_added_.covariance = Eigen::MatrixXd::Zero(6, 6);
  //
  main_graph->setFixedObjects(fixed_objects_);
}

void OptimizerG2O::updateOdomMapTransform()
{
  // Eigen::Isometry3d new_map_odom_tranform = getOptimizedPose() * last_odometry_added_.odometry.inverse() * earth_to_map_transform_.inverse();
  // return;
  //
  earth_map_transform_ = getOptimizedMapPose();
  // DEBUG(earth_map_transform_.translation());
  // Eigen::Vector3d euler_angles = earth_map_transform_.rotation().eulerAngles(0, 1, 2);
  // DEBUG(euler_angles);

  // auto actual_map_to_odom_transform = earth_to_map_transform_.inverse() * getOptimizedPose();


  // Eigen::Isometry3d new_map_odom_tranform = initial_earth_to_map_transform_.inverse() * getOptimizedPose() * last_odometry_added_.odometry.inverse();
  Eigen::Isometry3d new_map_odom_tranform = earth_map_transform_.inverse() * getOptimizedPose() * last_odometry_added_.odometry.inverse();
  // Eigen::Isometry3d map_odom_diff = new_map_odom_tranform.inverse() * map_odom_tranform_;
  // if (map_odom_diff.translation().norm() > map_odom_security_threshold_) {
  //   WARN("Map-Odom transform difference is too big: " << map_odom_diff.translation().norm());
  //   return;
  // }
  map_odom_tranform_ = new_map_odom_tranform;
}

Eigen::Isometry3d OptimizerG2O::getOptimizedPose()
{
  return main_graph->getLastOdomNode()->getPose();
}

Eigen::Isometry3d OptimizerG2O::getOptimizedMapPose()
{
  return main_graph->getMapNode()->getPose();
}

Eigen::Isometry3d OptimizerG2O::getMapOdomTransform()
{
  return map_odom_tranform_;
}

Eigen::Isometry3d OptimizerG2O::getMapTransform()
{
  return earth_map_transform_;
}
