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
  CLOG(DEBUG, "mpc.follower") << "Requested interpolation time " << time;

  auto up_it = path_info_.lower_bound(time);
  CLOG(DEBUG, "mpc.follower") << "Found lower bound time " << up_it->first;

  if (up_it == path_info_.begin()){
    CLOG(ERROR, "mpc.follower") << "Finding first element of map!";    
    up_it++;
  } else if (up_it == path_info_.end()){ 
    CLOG(ERROR, "mpc.follower") << "Finding last element of map!";

    up_it--;
    const Transformation T_w_p0 = up_it->second;
    const double t_0 = up_it->first;

    return T_w_p0;
  }
  
  const Transformation T_w_p1 = up_it->second;
  const double t_1 = up_it->first;

  up_it--;
  const Transformation T_w_p0 = up_it->second;
  const double t_0 = up_it->first;

  const double dt = t_1 - t_0;
  if (abs(dt) > 1e9) {
    CLOG(WARNING, "mpc.follower") << "Ignoring interpolation due to long delta t!";
    return T_w_p0;
  }
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

tactic::Timestamp PathInterpolator::start() const {
  return path_info_.begin()->first;
}


PoseResultHomotopy generateFollowerReferencePosesEuclidean(const TransformList& leader_world_poses, const double final_leader_p_value, const tactic::LocalizationChain::Ptr chain, double robot_p, double target_distance) {
  PoseResultHomotopy follower_reference;
  
  // Run through the path and find the pose that best fulfills the distance constraint
  std::vector<double> best_distance(leader_world_poses.size(), std::numeric_limits<double>::max());
  std::vector<double> best_width(leader_world_poses.size(), std::numeric_limits<double>::max());
  std::vector<int> leader_pose_done(leader_world_poses.size(), 0);
  std::vector<lgmath::se3::Transformation> best_pose(leader_world_poses.size());

  for(double p = robot_p; p < final_leader_p_value; p += 0.02) {
    tactic::SegmentInfo closestSegment = findClosestSegment(p, chain, chain->trunkSequenceId());
    double interp = std::clamp((p - closestSegment.start_p) / (chain->p(closestSegment.end_sid) - closestSegment.start_p), 0.0, 1.0);
    lgmath::se3::Transformation pose = interpolatePoses(interp, chain->pose(closestSegment.start_sid), chain->pose(closestSegment.end_sid));

    
    for (uint i = 0; i < leader_world_poses.size(); i++){

      // Check this pose if we are not already beyond it
      if(leader_pose_done[i] == 0)
      {
        // Leader pose in world frame
        auto T_w_l = leader_world_poses[i];
        double dist = (pose.inverse() * T_w_l).r_ab_inb().norm();
        if (fabs(dist - target_distance) < best_distance[i]) {
          best_distance[i] = fabs(dist - target_distance);
          best_pose[i] = pose;
          auto width1 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.start_sid));
          auto width2 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.end_sid));
          best_width[i] = (1-interp) * width1 + interp * width2;
        }
        if (dist < 0.10) {
          // We are close enough to the leader pose, we can stop checking further
          leader_pose_done[i] = 1;
        }
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