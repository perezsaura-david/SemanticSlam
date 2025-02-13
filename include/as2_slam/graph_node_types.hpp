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
 *  \file       graph_node_types.hpp
 *  \brief      An state estimation server for AeroStack2
 *  \authors    David Pérez Saura
 *              Miguel Fernández Cortizas
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 ********************************************************************************/

#ifndef AS2_SLAM__GRAPH_NODE_TYPES_HPP_
#define AS2_SLAM__GRAPH_NODE_TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

#include <string>

#include <visualization_msgs/msg/marker.hpp>
#include "utils/conversions.hpp"

class GraphNode
{
public:
  virtual g2o::HyperGraph::Vertex * getVertex()            = 0;
  virtual visualization_msgs::msg::Marker getVizMarker()   = 0;
  virtual Eigen::Vector4d getVizMarkerColor()              = 0;
  virtual std::string getVizMarkerNamespace()              = 0;
  virtual std::string getNodeName()                        = 0;
  virtual Eigen::MatrixXd getOptimizedInformationMatrix()  = 0;
};

class GraphNodePoint3D : public GraphNode
{
public:
  explicit GraphNodePoint3D(const Eigen::Vector3d & _position)
  {
    vertex_ = new g2o::VertexPointXYZ();
    vertex_->setEstimate(_position);
  }
  ~GraphNodePoint3D() {}

  g2o::HyperGraph::Vertex * getVertex() override
  {
    return static_cast<g2o::HyperGraph::Vertex *>(vertex_);
  }

  g2o::VertexPointXYZ * getVertexPoint3D() {return vertex_;}

  visualization_msgs::msg::Marker getVizMarker() override
  {
    visualization_msgs::msg::Marker node_marker_msg;
    node_marker_msg.type = node_marker_msg.SPHERE;
    node_marker_msg.ns = getVizMarkerNamespace();
    node_marker_msg.id = vertex_->id();
    node_marker_msg.pose = convertToGeometryMsgPose(getPosition());
    node_marker_msg.scale.x = 0.5;
    node_marker_msg.scale.y = 0.5;
    node_marker_msg.scale.z = 0.5;
    Eigen::Vector4d color = getVizMarkerColor();
    node_marker_msg.color.r = color[0];
    node_marker_msg.color.g = color[1];
    node_marker_msg.color.b = color[2];
    node_marker_msg.color.a = color[3];
    return node_marker_msg;
  }

  virtual void setFixed() {vertex_->setFixed(true);}
  Eigen::Vector3d getPosition() {return vertex_->estimate();}
  void setCovariance(const Eigen::MatrixXd & _cov_matrix) {cov_matrix_ = _cov_matrix;}
  Eigen::MatrixXd getCovariance() {return cov_matrix_;}
  // Eigen::MatrixXd getInformationMatrix() override {return vertex_->A();}  // Hessian matrix block
  Eigen::MatrixXd getOptimizedInformationMatrix() override {return vertex_->A();}  // Hessian matrix block

protected:
  std::string getNodeName() override {return node_name_;}
  std::string getVizMarkerNamespace() override
  {
    return element_name_ + "/" + getNodeName();
  }
  Eigen::Vector4d getVizMarkerColor() override {return viz_color_;}

  g2o::VertexPointXYZ * vertex_;
  std::string element_name_ = "node";
  std::string node_name_ = "Point3D";
  Eigen::Vector4d viz_color_ = {1.0, 1.0, 1.0, 1.0};
  Eigen::MatrixXd cov_matrix_;
};

class GraphNodeSE3 : public GraphNode
{
public:
  explicit GraphNodeSE3(const Eigen::Isometry3d & _pose)
  {
    vertex_ = new g2o::VertexSE3();
    vertex_->setEstimate(_pose);
  }
  ~GraphNodeSE3() {}

  g2o::HyperGraph::Vertex * getVertex() override
  {
    return static_cast<g2o::HyperGraph::Vertex *>(vertex_);
  }

  g2o::VertexSE3 * getVertexSE3() {return vertex_;}
  Eigen::MatrixXd getOptimizedInformationMatrix() override {return vertex_->A();}  // Hessian matrix block

  visualization_msgs::msg::Marker getVizMarker() override
  {
    visualization_msgs::msg::Marker node_marker_msg;
    node_marker_msg.type = node_marker_msg.ARROW;
    node_marker_msg.ns = getVizMarkerNamespace();
    node_marker_msg.id = vertex_->id();
    node_marker_msg.pose = convertToGeometryMsgPose(getPose());
    node_marker_msg.scale.x = 0.5;
    node_marker_msg.scale.y = 0.05;
    node_marker_msg.scale.z = 0.05;
    Eigen::Vector4d color = getVizMarkerColor();
    node_marker_msg.color.r = color[0];
    node_marker_msg.color.g = color[1];
    node_marker_msg.color.b = color[2];
    node_marker_msg.color.a = color[3];
    return node_marker_msg;
  }

  virtual void setFixed() {vertex_->setFixed(true);}
  Eigen::Isometry3d getPose() {return vertex_->estimate();}
  void setCovariance(const Eigen::MatrixXd & _cov_matrix) {cov_matrix_ = _cov_matrix;}
  Eigen::MatrixXd getCovariance() {return cov_matrix_;}

protected:
  std::string getNodeName() override {return node_name_;}
  std::string getVizMarkerNamespace() override
  {
    return element_name_ + "/" + getNodeName();
  }
  Eigen::Vector4d getVizMarkerColor() override {return viz_color_;}

  g2o::VertexSE3 * vertex_;
  std::string element_name_ = "node";
  std::string node_name_ = "SE3";
  Eigen::Vector4d viz_color_ = {1.0, 1.0, 1.0, 1.0};
  Eigen::MatrixXd cov_matrix_;
};

class GateNode : public GraphNodePoint3D
{
public:
  explicit GateNode(const Eigen::Vector3d & _position)
  : GraphNodePoint3D(_position) {}

protected:
  std::string node_name_ = "Gate";
  Eigen::Vector4d viz_color_ = {0.0, 1.0, 1.0, 1.0};
  std::string getNodeName() override {return node_name_;}
  Eigen::Vector4d getVizMarkerColor() override {return viz_color_;}
};

class ArucoNode : public GraphNodeSE3
{
public:
  explicit ArucoNode(const Eigen::Isometry3d & _pose)
  : GraphNodeSE3(_pose) {}

protected:
  std::string node_name_ = "Aruco";
  Eigen::Vector4d viz_color_ = {0.0, 1.0, 0.0, 1.0};
  std::string getNodeName() override {return node_name_;}
  Eigen::Vector4d getVizMarkerColor() override {return viz_color_;}
};

class OdomNode : public GraphNodeSE3
{
public:
  explicit OdomNode(const Eigen::Isometry3d & _pose)
  : GraphNodeSE3(_pose) {}

protected:
  std::string node_name_ = "Odometry";
  Eigen::Vector4d viz_color_ = {0.0, 0.0, 1.0, 1.0};
  std::string getNodeName() override {return node_name_;}
  Eigen::Vector4d getVizMarkerColor() override {return viz_color_;}
};

#endif  // AS2_SLAM__GRAPH_NODE_TYPES_HPP_
