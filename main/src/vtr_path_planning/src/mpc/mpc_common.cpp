// Copyright 2024, Autonomous Space Robotics Lab (ASRL)
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
 * \file mpc_common.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */

#include "vtr_path_planning/mpc/mpc_common.hpp"
#include <vtr_tactic/types.hpp>



namespace vtr::path_planning {
tactic::EdgeTransform tf_from_global(double x, double y,
                           double theta) {
    auto rotm = lgmath::so3::vec2rot({0, 0, theta});
    Eigen::Vector3d final_pose{x, y, 0};
    return tactic::EdgeTransform(rotm, -rotm.transpose() * final_pose);
}

CurvatureInfo CurvatureInfo::fromTransform(const lgmath::se3::Transformation& T) {
  // Note that this is only along a relative path with an origin at 0,0
  // Using the base tf is still required to move into the world frame
  auto aang = lgmath::so3::rot2vec(T.C_ba());
  double roc = T.r_ab_inb().norm() / 2 / (sin(aang(2) / 2) + sgn(sin(aang(2) / 2)) * 1e-6);

  Eigen::Matrix3d rotm_perp;
  rotm_perp << 0.0, -sgn(roc), 0.0, sgn(roc), 0.0, 0.0, 0.0, 0.0, 1.0;

  auto dist = T.r_ab_inb().norm();
  auto lin_dir = T.r_ab_inb() / dist;

  Eigen::Vector3d coc = T.r_ab_inb() / 2 + sqrt(roc * roc - dist * dist / 4) *
                                                rotm_perp * lin_dir;
  return {coc, roc};
}

tactic::Direction findDirection(const lgmath::se3::Transformation& T_wr, const tactic::LocalizationChain::Ptr chain, const unsigned sid, const unsigned next_sid) {
    auto unit_vec = Eigen::Vector2d(cos(lgmath::so3::rot2vec(T_wr.C_ba())(2)),
                    sin(lgmath::so3::rot2vec(T_wr.C_ba())(2)) );
    auto dir_vector = (chain->pose(next_sid).r_ab_inb() - chain->pose(sid).r_ab_inb()).head<2>();

    tactic::Direction dir = tactic::Direction::Unknown;
    if (unit_vec.dot(dir_vector) < 0){
      dir = tactic::Direction::Backward;
    }
    else {
      dir = tactic::Direction::Forward;
    }
    return dir;
}

tactic::SegmentInfo findClosestSegment(const lgmath::se3::Transformation& T_wr, const tactic::LocalizationChain::Ptr chain, unsigned sid_start) {
    double best_distance = std::numeric_limits<double>::max();
    double max_distance = -1.;
    unsigned best_sid = sid_start;
    const unsigned end_sid = std::min(sid_start + 20 + 1,
                                    unsigned(chain->size()));
    bool direction_switch = false;
    // Explicit casting to avoid numerical underflow when near the beginning of
    // the chain
    const unsigned begin_sid = unsigned(std::max(int(sid_start) - 5, 0));

    // Find the closest vertex to the input
    for (auto path_it = chain->begin(begin_sid); unsigned(path_it) < end_sid;
        ++path_it) {

      // Calculate the distance
      double distance = (T_wr.inverse() * chain->pose(path_it)).r_ab_inb().norm();
      // CLOG(DEBUG, "mpc.cost_function") << "Dist: " << distance << " sid: " << unsigned(path_it);

      // Record the best distance
      max_distance = std::max(distance, max_distance);
      if (distance < best_distance) {
        best_distance = distance;
        best_sid = unsigned(path_it);
      }

      // This block detects direction switches, and prevents searching across them
      if (unsigned(path_it) < end_sid - 1) {
        Eigen::Matrix<double, 6, 1> vec_prev_cur = path_it->T().vec();
        Eigen::Matrix<double, 6, 1> vec_cur_next = (path_it + 1)->T().vec();
        // + means they are in the same direction (note the negative at the front
        // to invert one of them)
        double r_dot = vec_prev_cur.head<3>().dot(vec_cur_next.head<3>());
        // + means they are in the same direction
        double C_dot = vec_prev_cur.tail<3>().dot(vec_cur_next.tail<3>());
        // combine the translation and rotation components using the angle weight
        double T_dot = r_dot + 0.25 * C_dot;
        // If this is negative, they are in the 'opposite direction', and we're at
        // a cusp
        if (T_dot < 0) {
          if (unsigned(path_it) <= sid_start) {
            CLOG(DEBUG, "cbit.debug") << "Direction switch behind reset";
            best_distance = std::numeric_limits<double>::max();
            max_distance = -1.;
          } else {
            CLOG(DEBUG, "cbit.debug") << "Direction switch ahead break";
            direction_switch = true;
            break;
          }
        }
      }
    }


    // Assume forward at end of path
    auto dir = tactic::Direction::Unknown;

    //Handle end of path exceptions
    if(best_sid == 0){
      dir = findDirection(T_wr, chain, best_sid, best_sid + 1);
      auto seg_info = tactic::SegmentInfo(dir, best_sid, best_sid + 1); 
      seg_info.direction_switch = direction_switch;
      return seg_info;
    }
    if(best_sid == chain->size() - 1){
      dir = findDirection(T_wr, chain, best_sid - 1, best_sid);
      auto seg_info = tactic::SegmentInfo(dir, best_sid-1, best_sid); 
      seg_info.direction_switch = direction_switch;
      return seg_info;
    }

    auto curr_dir = (chain->pose(best_sid).inverse() * T_wr).r_ab_inb();
    auto next_dir = (chain->pose(best_sid).inverse() * chain->pose(best_sid + 1)).r_ab_inb();

    if(curr_dir.dot(next_dir) > 0){
      dir = findDirection(T_wr, chain, best_sid, best_sid + 1);
      auto seg_info = tactic::SegmentInfo(dir, best_sid, best_sid + 1); 
      seg_info.direction_switch = direction_switch;
      return seg_info;
    }
    else {
      dir = findDirection(T_wr, chain, best_sid - 1, best_sid);
      auto seg_info = tactic::SegmentInfo(dir, best_sid-1, best_sid); 
      seg_info.direction_switch = direction_switch;
      return seg_info;
    }
  }

tactic::SegmentInfo findClosestSegment(const double p, const tactic::LocalizationChain::Ptr chain, unsigned sid_start) {

    double best_distance = std::numeric_limits<double>::max();
    double max_distance = -1.;
    unsigned best_sid = sid_start;
    const unsigned end_sid = std::min(sid_start + 20 + 1,
                                    unsigned(chain->size()));

    // Explicit casting to avoid numerical underflow when near the beginning of
    // the chain
    const unsigned begin_sid = unsigned(std::max(int(sid_start) - 5, 0));

    // Find the closest vertex to the input
    for (auto path_it = chain->begin(begin_sid); unsigned(path_it) < end_sid;
        ++path_it) {

      // Calculate the distance
      double distance = abs(p - chain->p(path_it));
      // CLOG(DEBUG, "mpc.cost_function") << "Dist: " << distance << " sid: " << unsigned(path_it);

      // Record the best distance
      max_distance = std::max(distance, max_distance);
      if (distance < best_distance) {
        best_distance = distance;
        best_sid = unsigned(path_it);
      }
      
    }

    //Handle end of path exceptions
    if(best_sid == 0)
      return tactic::SegmentInfo(best_sid, best_sid + 1);
    if(best_sid == chain->size() - 1)
      return tactic::SegmentInfo(best_sid - 1, best_sid);

    if(p - chain->p(best_sid) > 0){
      return tactic::SegmentInfo(best_sid, best_sid + 1);
    }
    else{
      return tactic::SegmentInfo(best_sid - 1, best_sid);
    }
  }

std::pair<tactic::Direction, double> findRobotP(const lgmath::se3::Transformation& T_wr, const tactic::LocalizationChain::Ptr chain) {
  double state_interp = 0;
  auto segment = findClosestSegment(T_wr, chain, chain->trunkSequenceId());
  auto path_ref = interpolatePath(T_wr, chain->pose(segment.start_sid), chain->pose(segment.end_sid), state_interp);
  return std::make_pair(segment.dir, chain->p(segment.start_sid) + state_interp * (chain->p(segment.end_sid) - chain->p(segment.start_sid)));
}

lgmath::se3::Transformation interpolatePoses(const double interp,
                const lgmath::se3::Transformation& seq_start, const lgmath::se3::Transformation& seq_end) {
  const lgmath::se3::Transformation edge = seq_start.inverse() * seq_end;
  return seq_start * lgmath::se3::Transformation(interp * edge.vec(), 0);
}

lgmath::se3::Transformation interpolatePath(const lgmath::se3::Transformation& T_wr,
                const lgmath::se3::Transformation& seq_start, const lgmath::se3::Transformation& seq_end,
                 double& interp) {
  const lgmath::se3::Transformation edge = seq_start.inverse() * seq_end;
  const auto& [coc, roc] = CurvatureInfo::fromTransform(edge);
  Eigen::Vector4d coc_h{0, 0, 0, 1};

  if (abs(roc) < 30) {
    coc_h.head<3>() = coc;

    coc_h = seq_start.matrix() * coc_h;

    const auto interp_ang =
        acos(std::clamp((T_wr.r_ab_inb() - coc_h.head<3>())
                .normalized()
                .dot((seq_start.r_ab_inb() - coc_h.head<3>()).normalized()), -1.0, 1.0));

    const auto interp_full =
        acos(std::clamp((seq_end.r_ab_inb() - coc_h.head<3>())
                .normalized()
                .dot((seq_start.r_ab_inb() - coc_h.head<3>()).normalized()), -1.0, 1.0));

    interp = interp_ang / interp_full;
  } else {
    const auto edge_vec = seq_end.r_ab_inb() - seq_start.r_ab_inb();
    interp = (T_wr.r_ab_inb() - seq_start.r_ab_inb()).dot(edge_vec.normalized()) / edge_vec.norm();
  }

  // interp = std::clamp(interp, 0.0, 1.0);
  if (abs(interp) - std::clamp(interp, 0.0, 1.0) > 0.001) {
    CLOG(WARNING, "test") << "Extrapolating beyond path segment. Interp " << interp;
    CLOG(DEBUG, "test") << "CoC is " << coc << " RoC is " << roc;
    CLOG(DEBUG, "test") << "Position is " << T_wr.matrix();
    CLOG(DEBUG, "test") << "Start is " << seq_start.matrix();
    CLOG(DEBUG, "test") << "End is " << seq_end.matrix();
    CLOG(DEBUG, "test") << "Center is " << coc_h.head<3>();
  }
  if (std::isnan(interp)) { 
    throw std::runtime_error("ERROR NAN FOUND!!!!!");
  }
  return interpolatePoses(interp, seq_start, seq_end);
}


// For generating VT&R teach path poses used in the corridor mpc (new version which directly uses the interpolated p measurements from the cbit path trajectory tracking)
PoseResultHomotopy generateHomotopyReference(const std::vector<lgmath::se3::Transformation>& rolled_out_poses, tactic::LocalizationChain::Ptr chain) {

    // Initialize vectors storing the barrier values:
    std::vector<double> barrier_q_left;
    std::vector<double> barrier_q_right;

    std::vector<lgmath::se3::Transformation> tracking_reference_poses;

    unsigned last_sid = chain->trunkSequenceId();

    // Iterate through the interpolated p_measurements and make interpolate euclidean poses from the teach path
    for (const auto& T_wrk : rolled_out_poses) {
      auto closestSegment = findClosestSegment(T_wrk, chain, last_sid);
      last_sid = closestSegment.start_sid;

      double interp;
      auto interpTf = interpolatePath(T_wrk, chain->pose(closestSegment.start_sid), chain->pose(closestSegment.end_sid), interp);

      // add to measurement vector
      tracking_reference_poses.push_back(interpTf);

      // Find the corresponding left and right barrier q values to pass to the mpc
      auto width1 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.start_sid));
      auto width2 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.end_sid));
      barrier_q_left.push_back((1-interp) * width1 + interp * width2);
      barrier_q_right.push_back((1-interp) * width1 + interp * width2);
    }

    return {tracking_reference_poses, barrier_q_left, barrier_q_right};
}

// For generating VT&R teach path poses used in the corridor mpc (new version which directly uses the interpolated p measurements from the cbit path trajectory tracking)
PoseResultHomotopy generateHomotopyReference(const std::vector<double>& rolled_out_p, tactic::LocalizationChain::Ptr chain) {

    // Initialize vectors storing the barrier values:
    std::vector<double> barrier_q_max;
    std::vector<double> barrier_q_min;
    // Flag to indicate we need to fill the back of the vector
    bool direction_switch = false;

    std::vector<lgmath::se3::Transformation> tracking_reference_poses;
    unsigned last_sid = chain->trunkSequenceId();
    auto last_tf = chain->T_trunk_target(last_sid);
    auto last_dir = last_tf.vec().head<3>();

    // Iterate through the interpolated p_measurements and make interpolate euclidean poses from the teach path
    for (const auto& p_target : rolled_out_p) {
      auto closestSegment = findClosestSegment(p_target, chain, last_sid);
      
      double interp = std::clamp((p_target - chain->p(closestSegment.start_sid)) / (chain->p(closestSegment.end_sid) - chain->p(closestSegment.start_sid)), 0.0, 1.0);
      auto interpTf = interpolatePoses(interp, chain->pose(closestSegment.start_sid), chain->pose(closestSegment.end_sid));

      // add to measurement vector
      tracking_reference_poses.push_back(interpTf);

      // Find the corresponding left and right barrier q values to pass to the mpc
      auto width1 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.start_sid));
      auto width2 = pose_graph::BasicPathBase::terrian_type_corridor_width(chain->query_terrain_type(closestSegment.end_sid));
      barrier_q_max.push_back((1-interp) * width1 + interp * width2);
      barrier_q_min.push_back(-(1-interp) * width1 - interp * width2);

      // Detect a direction switch, as the switch may occur within a path segment. This gives us a better resolution
      // For the actual path
      if (closestSegment.start_sid < chain->size() - 1) {
        Eigen::Matrix<double, 6, 1> vec_prev = last_tf.vec(); 
        Eigen::Matrix<double, 6, 1> vec_curr = interpTf.vec();

        // Get the translation component between our node and the last node
        auto curr_dir = vec_curr.head<3>() - vec_prev.head<3>();

        // Check if our last and current translations are in opposing directions
        auto T_dot = curr_dir.dot(last_dir);
        last_dir = curr_dir;
        // a cusp
        if (T_dot < 0) {
          CLOG(DEBUG, "cbit.debug") << "Direction switch ahead break";
          direction_switch = true;
          tracking_reference_poses.pop_back();
          break;
        }
      }
      last_sid = closestSegment.start_sid;
      last_tf = interpTf;
    }

    if (direction_switch && tracking_reference_poses.size() != rolled_out_p.size()){
      CLOG(INFO, "cbit.debug") << "Direction switch detected, but not all poses were filled. Filling the back of the vector with the last pose.";
      // Fill the back of the vector with the last pose
      for (size_t i = tracking_reference_poses.size(); i < rolled_out_p.size(); ++i) {
        tracking_reference_poses.push_back(tracking_reference_poses.back());
        barrier_q_max.push_back(barrier_q_max.back());
        barrier_q_min.push_back(barrier_q_min.back());
      }
    }

    return {tracking_reference_poses, barrier_q_max, barrier_q_min};
}

lgmath::se3::Transformation interpolatedPose(double p, const tactic::LocalizationChain::Ptr chain) {
  auto closestSegment = findClosestSegment(p, chain, chain->trunkSequenceId());
  double interp = std::clamp((p - chain->p(closestSegment.start_sid)) / (chain->p(closestSegment.end_sid) - chain->p(closestSegment.start_sid)), 0.0, 1.0);
  return interpolatePoses(interp, chain->pose(closestSegment.end_sid), chain->pose(closestSegment.end_sid));
}
}