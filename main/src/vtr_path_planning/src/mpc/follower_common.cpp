// Copyright 2025, Autonomous Space Robotics Lab (ASRL)
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
 * \file follower_common.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */

#include <vtr_path_planning/mpc/follower_common.hpp>

namespace vtr::path_planning
{
PathInterpolator::PathInterpolator(const nav_msgs::msg::Path::SharedPtr& path) {
  using namespace vtr::common::conversions;
  if (path->poses.size() == 0)
    throw std::range_error("Path length cannot be 0 for interpolation!");
  
  for(const auto& pose : path->poses) {
    path_info_[rclcpp::Time(pose.header.stamp).nanoseconds()] = tfFromPoseMessage(pose.pose);
  }
}

PathInterpolator::Transformation PathInterpolator::at(tactic::Timestamp time) const {
  auto up_it = path_info_.lower_bound(time);

  CLOG(DEBUG, "mpc.follower") << "Requested interpolation time " << time;
  CLOG(DEBUG, "mpc.follower") << "Found lower bound time " << up_it->first;

  if (up_it == path_info_.begin()){
    CLOG(ERROR, "mpc.follower") << "Finding first element of map!";
    const Transformation T_w_p1 = up_it->second;
    const double t_1 = up_it->first;
    
    up_it--;
    const Transformation T_w_p0 = up_it->second;
    const double t_0 = up_it->first;
    CLOG(DEBUG, "mpc.follower") << "Time 1 " << t_1 << " Time 0 " << t_0;

    return T_w_p1;
  } else if (up_it == path_info_.end()){ 
    CLOG(ERROR, "mpc.follower") << "Finding last element of map!";

    up_it--;
    const Transformation T_w_p0 = up_it->second;
    const double t_0 = up_it->first;

    return T_w_p0;
  } else {
    const Transformation T_w_p1 = up_it->second;
    const double t_1 = up_it->first;

    up_it--;
    const Transformation T_w_p0 = up_it->second;
    const double t_0 = up_it->first;


    //CLOG(DEBUG, "mpc.follower") << "Pose 0 " << T_w_p0 << " Pose 1 " << T_w_p1;

    const double dt = t_1 - t_0;
    try {
      auto xi_interp = (T_w_p0.inverse() * T_w_p1).vec() * ((time - t_0) / dt);
      //CLOG(DEBUG, "mpc.follower") << "delta trans " << (T_w_p0.inverse() * T_w_p1).vec();
      //CLOG(DEBUG, "mpc.follower") << "dt " << dt;
      //CLOG(DEBUG, "mpc.follower") << "time " << time;
      //CLOG(DEBUG, "mpc.follower") << "t_0 " << t_0;
      //CLOG(DEBUG, "mpc.follower") << "xi" << xi_interp;
      //CLOG(DEBUG, "mpc.follower") << "Interpolated pose " << T_w_p0 * Transformation(Eigen::Matrix<double, 6, 1>(xi_interp));
      return T_w_p0 * Transformation(Eigen::Matrix<double, 6, 1>(xi_interp));
    } catch (std::exception &e) {
      return T_w_p0;
    }
  }
}

tactic::Timestamp PathInterpolator::start() const {
  return path_info_.begin()->first;
}


PoseResultHomotopy generateFollowerReferencePosesEuclidean(const TransformList& leader_world_poses, const double final_leader_p_value, const tactic::LocalizationChain::Ptr chain, double robot_p, double target_distance) {
  PoseResultHomotopy follower_reference;
  
  // Run through the path and find the pose that best fulfills the distance constraint
  std::vector<double> best_distance(leader_world_poses.size(), std::numeric_limits<double>::max());
  std::vector<double> best_width(leader_world_poses.size(), std::numeric_limits<double>::max());
  std::vector<lgmath::se3::Transformation> best_pose(leader_world_poses.size());

  for(double p = robot_p; p < final_leader_p_value; p += 0.02) {
    Segment closestSegment = findClosestSegment(p, chain, chain->trunkSequenceId());
    double interp = std::clamp((p - chain->p(closestSegment.first)) / (chain->p(closestSegment.second) - chain->p(closestSegment.first)), 0.0, 1.0);
    lgmath::se3::Transformation pose = interpolatePoses(interp, chain->pose(closestSegment.first), chain->pose(closestSegment.second));

    
    for (uint i = 0; i < leader_world_poses.size(); i++){

      // Check this pose if we are not already beyond it
        // Leader pose in world frame
        auto T_w_l = leader_world_poses[i];
        double dist = (pose.inverse() * T_w_l).r_ab_inb().norm();
        if (fabs(dist - target_distance) < best_distance[i]) {
          best_distance[i] = fabs(dist - target_distance);
          best_pose[i] = pose;
          auto width1 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.first));
          auto width2 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.second));
          best_width[i] = (1-interp) * width1 + interp * width2;
        }

    }
  }
  follower_reference.poses = best_pose;
  for (const auto& width : best_width){
    follower_reference.barrier_q_min.push_back(-width);
    follower_reference.barrier_q_max.push_back(width);
  }

  return follower_reference;
}


} // namespace vtr::path_planning