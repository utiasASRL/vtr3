// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file lateral_error_evaluator.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/lateral_error_evaluators.hpp"
#include <iostream>
#include "math.h"
#include "vtr_path_planning/cbit/utils.hpp"


namespace vtr {
namespace steam_extension {

  CurvatureInfo CurvatureInfo::fromTransform(const Transformation &T) {
    auto aang = lgmath::so3::rot2vec(T.inverse().C_ba());
    double roc = T.r_ba_ina().norm() / 2 / (sin(aang(2) / 2) + 1e-6);
    CLOG(DEBUG, "mpc.debug") << "Radius of Curvature " << roc;

    static Eigen::Matrix3d rotm_perp; 
    rotm_perp << 0.0, -1.0, 0.0,
                 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0;

    auto dist = T.r_ba_ina().norm();
    auto lin_dir = T.r_ba_ina() / dist;

    Eigen::Vector3d coc = T.r_ba_ina() / 2 + sqrt(roc*roc - dist*dist / 4) * sgn(roc) * rotm_perp * lin_dir;
    return {coc, roc};
  }

  LateralErrorEvaluator::Ptr LateralErrorEvaluator::MakeShared(const Evaluable<InType>::ConstPtr& xi,
                          const PathPtr path) {
    return std::make_shared<LateralErrorEvaluator>(xi, path);
  }


  bool LateralErrorEvaluator::active() const { return xi_->active(); }

  void LateralErrorEvaluator::getRelatedVarKeys(KeySet& keys) const {
    xi_->getRelatedVarKeys(keys);
  }

  LateralErrorEvaluator::OutType LateralErrorEvaluator::value() const {
    //Find path segment
    auto segment = findClosestSegment(tf_->value());

    const auto &[coc, roc] = CurvatureInfo::fromTransform(
                              path_->pose(segment.first).inverse() * path_->pose(segment.second));

    const auto value = (xi_->value().head<2>() - coc.head<2>()).norm() - roc;
    OutType m_val;
    m_val(0, 0) = value;
    return m_val;
  }

  typename steam::Node<LateralErrorEvaluator::OutType>::Ptr LateralErrorEvaluator::forward() const {
    const auto child = xi_->forward();

    auto segment = findClosestSegment(tf_->value());

    CLOG(DEBUG, "mpc.cost_function") << "Segment IDs: "  << segment;
    CLOG(DEBUG, "mpc.cost_function") << "Xi: " << child->value();

    const auto curve_info  = CurvatureInfo::fromTransform(
                              path_->pose(segment.first).inverse() * path_->pose(segment.second));
    const auto value = (child->value().head<2>() - curve_info.center.head<2>()).norm() - curve_info.radius;
    OutType m_val;
    m_val(0, 0) = value;
    const auto node = steam::Node<OutType>::MakeShared(m_val);
    node->addChild(child);

    //In case the localization chain changes between forwards and backwards pass
    //Store the info about the curve used for the error
    const auto curve_node = steam::Node<CurvatureInfo>::MakeShared(curve_info);
    node->addChild(curve_node);
    return node;
  }

  void LateralErrorEvaluator::backward(const Eigen::MatrixXd& lhs,
                                       const steam::Node<OutType>::Ptr& node,
                                       Jacobians& jacs) const {
    if (xi_->active()) {
      const auto child = std::static_pointer_cast<steam::Node<Eigen::Matrix<double, 6, 1>>>(node->at(0));
      const auto curvature_info = std::static_pointer_cast<steam::Node<CurvatureInfo>>(node->at(1));
      Eigen::Matrix<double, 1, 6> local_jac;
      local_jac << child->value()(0) - curvature_info->value().center(0),
                  child->value()(1) - curvature_info->value().center(1),
                  0, 0, 0, 0;

      Eigen::MatrixXd new_lhs = lhs * local_jac / (node->value()(0, 0) + curvature_info->value().radius);
      xi_->backward(new_lhs, child, jacs);
    }
  }

  LateralErrorEvaluator::Segment LateralErrorEvaluator::findClosestSegment(const Transformation T_rw) const {
  
    double best_distance = std::numeric_limits<double>::max();
    double max_distance = -1.;
    unsigned best_sid = last_closest_sid_;
    const unsigned end_sid = std::min(last_closest_sid_ + 20 + 1,
                                    unsigned(path_->size()));

    // Explicit casting to avoid numerical underflow when near the beginning of
    // the chain
    const unsigned begin_sid = unsigned(std::max(int(last_closest_sid_) - 5, 0));

    // Find the closest vertex to the input
    for (auto path_it = PathIter(path_.get(), begin_sid); unsigned(path_it) < end_sid;
        ++path_it) {

      // Calculate the distance
      double distance = (T_rw * path_->pose(path_it)).r_ab_inb().norm();
      // CLOG(DEBUG, "mpc.cost_function") << "Dist: " << distance << " sid: " << unsigned(path_it);

      // Record the best distance
      max_distance = std::max(distance, max_distance);
      if (distance < best_distance) {
        best_distance = distance;
        best_sid = unsigned(path_it);
      }

      // This block detects direction switches, and prevents searching across them
      // It's only enabled in safe-search mode (no searching backwards as well),
      // it only tries if the current sid is not an extremum of the search range,
      // and it only stops at cusps that pass X m in 'distance' from the current
      // position
      // if (max_distance > config_.min_cusp_distance &&
      //     unsigned(path_it) > begin_sid && unsigned(path_it) + 1 < end_sid) {
      //   Eigen::Matrix<double, 6, 1> vec_prev_cur = path_it->T().vec();
      //   Eigen::Matrix<double, 6, 1> vec_cur_next = (path_it + 1)->T().vec();
      //   // + means they are in the same direction (note the negative at the front
      //   // to invert one of them)
      //   double r_dot = vec_prev_cur.head<3>().dot(vec_cur_next.head<3>());
      //   // + means they are in the same direction
      //   double C_dot = vec_prev_cur.tail<3>().dot(vec_cur_next.tail<3>());
      //   // combine the translation and rotation components using the angle weight
      //   double T_dot = r_dot + config_.angle_weight * C_dot;
      //   // If this is negative, they are in the 'opposite direction', and we're at
      //   // a cusp
      //   if (T_dot < 0) {
      //     CLOG_EVERY_N(1, DEBUG, "pose_graph")
      //         << "Not searching past the cusp at " << path_it->to() << ", "
      //         << distance << " (m/8degress) away.";
      //     break;
      //   }
      // }
    }

    // last_closest_sid_ = best_sid;


    //Handle end of path exceptions
    if(best_sid == 0)
      return std::make_pair(best_sid, best_sid + 1);
    if(best_sid == path_->size())
      return std::make_pair(best_sid - 1, best_sid);

    double next_distance = (T_rw * path_->pose(best_sid + 1)).r_ab_inb().norm();
    double past_distance = (T_rw * path_->pose(best_sid - 1)).r_ab_inb().norm();

    if(next_distance < past_distance)
      return std::make_pair(best_sid, best_sid + 1);
    else
      return std::make_pair(best_sid - 1, best_sid);
  }

  LateralErrorEvaluator::Ptr path_track_error(const Evaluable<LateralErrorEvaluator::InType>::ConstPtr& tf,
                          const LateralErrorEvaluator::PathPtr path) {
                            return LateralErrorEvaluator::MakeShared(tf, path);
                          } 

}
}


namespace steam {

auto LateralErrorEvaluatorRight::MakeShared(const Evaluable<InType>::ConstPtr& pt,
                                         const InType& meas_pt) -> Ptr {
  return std::make_shared<LateralErrorEvaluatorRight>(pt, meas_pt);
}

LateralErrorEvaluatorRight::LateralErrorEvaluatorRight(
    const Evaluable<InType>::ConstPtr& pt, const InType& meas_pt)
    : pt_(pt), meas_pt_(meas_pt) {
  //D_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  D_(0,1) = 1;

}

bool LateralErrorEvaluatorRight::active() const { return pt_->active(); }

void LateralErrorEvaluatorRight::getRelatedVarKeys(KeySet& keys) const {
  pt_->getRelatedVarKeys(keys);
}

auto LateralErrorEvaluatorRight::value() const -> OutType {
  return D_ * (pt_->value() - meas_pt_);
}

auto LateralErrorEvaluatorRight::forward() const -> Node<OutType>::Ptr {
  const auto child = pt_->forward();
  const auto value = D_ * (child->value() - meas_pt_);
  // Debug check for Barrier Constraint
  //if (value <= 0.0)
  //{
  //  std::cout << "WARNING: MPC Crossed Lateral Barrier Constraints!" << value << std::endl;
  //}
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

void LateralErrorEvaluatorRight::backward(const Eigen::MatrixXd& lhs,
                                       const Node<OutType>::Ptr& node,
                                       Jacobians& jacs) const {
  if (pt_->active()) {
    const auto child = std::static_pointer_cast<Node<InType>>(node->at(0));
    Eigen::MatrixXd new_lhs = lhs * D_;
    pt_->backward(new_lhs, child, jacs);
  }
}

LateralErrorEvaluatorRight::Ptr homo_point_error_right(
    const Evaluable<LateralErrorEvaluatorRight::InType>::ConstPtr& pt,
    const LateralErrorEvaluatorRight::InType& meas_pt) {
  return LateralErrorEvaluatorRight::MakeShared(pt, meas_pt);
}




auto LateralErrorEvaluatorLeft::MakeShared(const Evaluable<InType>::ConstPtr& pt,
                                         const InType& meas_pt) -> Ptr {
  return std::make_shared<LateralErrorEvaluatorLeft>(pt, meas_pt);
}

LateralErrorEvaluatorLeft::LateralErrorEvaluatorLeft(
    const Evaluable<InType>::ConstPtr& pt, const InType& meas_pt)
    : pt_(pt), meas_pt_(meas_pt) {
  //D_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  D_(0,1) = 1;

}

bool LateralErrorEvaluatorLeft::active() const { return pt_->active(); }

void LateralErrorEvaluatorLeft::getRelatedVarKeys(KeySet& keys) const {
  pt_->getRelatedVarKeys(keys);
}

auto LateralErrorEvaluatorLeft::value() const -> OutType {
  return D_ * (meas_pt_ - pt_->value());
}

auto LateralErrorEvaluatorLeft::forward() const -> Node<OutType>::Ptr {
  const auto child = pt_->forward();
  const auto value = D_ * (meas_pt_ - child->value());
  const auto node = Node<OutType>::MakeShared(value);
  node->addChild(child);
  return node;
}

void LateralErrorEvaluatorLeft::backward(const Eigen::MatrixXd& lhs,
                                       const Node<OutType>::Ptr& node,
                                       Jacobians& jacs) const {
  if (pt_->active()) {
    const auto child = std::static_pointer_cast<Node<InType>>(node->at(0));
    Eigen::MatrixXd new_lhs = -lhs * D_;
    pt_->backward(new_lhs, child, jacs);
  }
}

LateralErrorEvaluatorLeft::Ptr homo_point_error_left(
    const Evaluable<LateralErrorEvaluatorLeft::InType>::ConstPtr& pt,
    const LateralErrorEvaluatorLeft::InType& meas_pt) {
  return LateralErrorEvaluatorLeft::MakeShared(pt, meas_pt);
}

}  // namespace steam