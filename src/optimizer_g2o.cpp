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

  // TODO(dps): Make this a parameter
  // std::string object_type = "gate";
  // std::string object_type = "aruco";
  // std::vector<std::pair<std::string, Eigen::Vector4d>> fixed_objects_list = {
  //   {"gate_1", Eigen::Vector4d(4.0, 1.3, 1.13, 3.14)},
  //   {"gate_2", Eigen::Vector4d(4.0, -1.34, 1.16, 0.0)},
  //   {"gate_3", Eigen::Vector4d(-4.0, -1.29, 1.16, 0.0)},
  //   {"gate_4", Eigen::Vector4d(-3.97, 1.28, 1.17, 3.14)}
  // };

  // std::vector<std::pair<std::string, Eigen::Vector4d>> fixed_objects_list = {
  //   {"gate_1", Eigen::Vector4d(5.6, 1.5, 1.375, 0.0)},
  //   {"gate_2", Eigen::Vector4d(14.3, -1.06, 1.375, 0.0)},
  // };

  // std::vector<FixedObject> fixed_objects;
  // for (auto object : fixed_objects_list) {
  //   FixedObject fixed_object;
  //   fixed_object.id = object.first;
  //   fixed_object.type = object_type;
  //   Eigen::Vector3d object_position = object.second.head(3);
  //   double yaw = object.second(3);
  //   // Convert yaw to quaternion
  //   Eigen::Quaterniond orientation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  //   fixed_object.isometry = convertToIsometry3d(object_position, orientation);
  //   fixed_objects.emplace_back(fixed_object);
  // }

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
  if (odometry_is_relative_) {
    // TODO(dps): RELATIVE ODOMETRY
    // relative_pose = odom_pose;
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
    return false;
  } else {
    // ABSOLUTE ODOMETRY
    _odometry_info.odom_ref = _new_odometry.odometry;
    _odometry_info.increment = _last_odometry_added.odometry.inverse() * _odometry_info.odom_ref;
    _odometry_info.covariance_matrix = _new_odometry.covariance;
    // _odometry_info.covariance_matrix = _new_odometry.covariance - _last_odometry_added.covariance;
  }
  // _odometry_info.map_ref = map_odom_tranform_ * _odometry_info.odom_ref;
  _odometry_info.map_ref = _odometry_info.odom_ref;

  if (_new_odometry.covariance.isZero()) {
    ERROR("Generated odometry covariance matrix is zero");
    return false;
  }

  // INFO(PRINT_VAR(_odometry_info.increment.translation().transpose()));
  // INFO(PRINT_VAR(_odometry_info.odom_ref.translation().transpose()));
  // INFO(PRINT_VAR(_odometry_info.map_ref.translation().transpose()));
  return true;
}

bool OptimizerG2O::handleNewOdom(
  const OdometryWithCovariance & _new_odometry)
{

  if (_new_odometry.covariance.isZero()) {
    ERROR("Received covariance matrix is zero");
    return false;
    // main_graph->initGraph(_new_odometry.odometry);
    // return true;
  }

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
  // INFO("New odometry distance is enough: " << new_odometry_info.increment.translation().norm());
  last_odometry_added_.odometry = new_odometry_info.odom_ref;
  last_odometry_added_.covariance = _new_odometry.covariance;

  // if (_new_odometry.covariance.isZero()) {
  //   ERROR("Covariance matrix is zero");
  //   main_graph->initGraph(_new_odometry.odometry);
  //   return true;
  // }

  // FLAG("ADDING NEW ODOMETRY TO MAIN GRAPH");
  // DEBUG(PRINT_VAR(new_odometry_info.odom_ref.translation().transpose()));
  // DEBUG(PRINT_VAR(new_odometry_info.increment.translation().transpose()));
  if (std::isnan(new_odometry_info.odom_ref.translation().x())) {
    // DEBUG(PRINT_VAR(_new_odometry.odometry.translation().transpose()));
    ERROR("Odometry is NaN");
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
          continue;
          // cov_matrix = main_graph_object_covariance;
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
        // INFO(PRINT_VAR(cov_matrix));

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

    // FLAG("RESET TEMP GRAPH");
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
    // WARN("New odometry distance is not enough: " << translation_distance_from_last_node);
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
  if (_detection_odometry.covariance.isZero()) {
    ERROR("Received detection covariance matrix is zero");
    return false;
  }
  if (!temp_graph_generated_) {
    last_detection_odometry_added_ = last_odometry_added_;
  }

  // OdometryInfo detection_odometry_info;
  generateOdometryInfo(
    _detection_odometry, last_detection_odometry_added_,
    _detection_odometry_info);

  if (!temp_graph_generated_) {
    temp_graph->initGraph(_detection_odometry_info.map_ref);
    temp_graph_generated_ = true;
    // main_graph_object_covariance = _object->getCovarianceMatrix();  // FIXME(dps): remove this
  } else {
    if (!checkAddingConditions(_detection_odometry_info, tmep_graph_odometry_distance_threshold_)) {
      // INFO(
      //   "New odometry distance is not enough: " <<
      //     detection_odometry_info.increment.translation().norm());
      return false;
    }
    // FLAG(
    //   "New odometry distance is enough: " <<
    //     detection_odometry_info.increment.translation().norm());
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
  odometry_is_relative_ = _params.odometry_is_relative;
  generate_odom_map_transform_ = _params.generate_odom_map_transform;
  fixed_objects_ = _params.fixed_objects;
  main_graph->setFixedObjects(fixed_objects_);
}

void OptimizerG2O::updateOdomMapTransform()
{
  map_odom_tranform_ = getOptimizedPose() * last_odometry_added_.odometry.inverse();
}

Eigen::Isometry3d OptimizerG2O::getOptimizedPose()
{
  return main_graph->getLastOdomNode()->getPose();
}

Eigen::Isometry3d OptimizerG2O::getMapOdomTransform()
{
  return map_odom_tranform_;
}
