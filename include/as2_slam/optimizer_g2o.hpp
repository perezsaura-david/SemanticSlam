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
 *  \file       optimizer_g2o.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef AS2_SLAM__OPTIMIZER_G2O_HPP_
#define AS2_SLAM__OPTIMIZER_G2O_HPP_

#include <Eigen/Dense>
#include <Eigen/src/Geometry/Transform.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <memory>

#include "as2_slam/graph_g2o.hpp"
#include "as2_slam/object_detection_types.hpp"
#include "utils/conversions.hpp"
#include "utils/general_utils.hpp"

struct OptimizerG2OParameters
{
  double main_graph_odometry_distance_threshold;     // meters
  double main_graph_odometry_orientation_threshold;  // radians
  double temp_graph_odometry_distance_threshold;     // meters
  double temp_graph_odometry_orientation_threshold;  // radians
  bool odometry_is_relative;
  bool generate_odom_map_transform;
  std::vector<FixedObject> fixed_objects;
};

class OptimizerG2O
{
public:
  OptimizerG2O();
  ~OptimizerG2O() {}
  std::shared_ptr<GraphG2O> main_graph;
  std::shared_ptr<GraphG2O> temp_graph;
  void setParameters(const OptimizerG2OParameters & _params);
  Eigen::Isometry3d getOptimizedPose();
  Eigen::Isometry3d getMapOdomTransform();

  bool handleNewOdom(
    const OdometryWithCovariance & _new_odometry);
  void handleNewObjectDetection(
    ObjectDetection * _object,
    const OdometryInfo & _detection_odometry_info);
  bool generateOdometryInfo(
    const OdometryWithCovariance & _new_odometry,
    const OdometryWithCovariance & _last_odometry_added,
    OdometryInfo & _odometry_info);
  void updateOdomMapTransform();
  bool checkAddingNewDetection(
    const OdometryWithCovariance & _detection_odometry,
    OdometryInfo & _detection_odometry_info);
  bool checkAddingConditions(
    const OdometryInfo & _odometry, const double distance_threshold);

private:
  bool first_odom_ = true;
  bool temp_graph_generated_ = false;
  bool init_main_graph_ = true;
  // TODO(dps): add time_threshold_
  double translation_distance_from_last_node_ = 0.0;
  double rotation_distance_from_last_node_ = 0.0;
  OdometryWithCovariance last_odometry_added_;
  OdometryWithCovariance last_detection_odometry_added_;
  Eigen::Isometry3d map_odom_tranform_;
  Eigen::MatrixXd main_graph_object_covariance;

  // PARAMETERS
  double main_graph_odometry_distance_threshold_ = 1.0;     // meters
  double main_graph_odometry_orientation_threshold_ = 1.0;  // radians
  double tmep_graph_odometry_distance_threshold_ = 0.1;     // meters
  double temp_graph_odometry_orientation_threshold_ = 0.1;  // radians
  bool odometry_is_relative_ = false;
  bool generate_odom_map_transform_ = false;
  std::vector<FixedObject> fixed_objects_;
};

#endif  // AS2_SLAM__OPTIMIZER_G2O_HPP_
