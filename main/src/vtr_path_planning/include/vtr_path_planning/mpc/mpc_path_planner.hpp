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
 * \file mpc_path_planner.hpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <casadi/casadi.hpp>
#include <vtr_tactic/types.hpp>
#include <lgmath.hpp>


namespace vtr::path_planning {
class CasadiUnicycleMPC {
public:
  PTR_TYPEDEFS(CasadiUnicycleMPC);
  using DM = casadi::DM;

  struct Config {
    // These values are defined the python code and exported
    // TODO add an automatic way to keep the code in sync
    static constexpr int nStates = 3;
    static constexpr int nControl = 2;
    static constexpr double alpha = 0.8;
    static constexpr int N = 15;
    static constexpr double DT = 0.2;
    DM previous_vel{nControl, 1};
    DM T0{nStates, 1};
    std::vector<DM> reference_poses;
    std::vector<double> up_barrier_q;
    std::vector<double> low_barrier_q;
    double VF = 0.0;
    DM vel_max{nControl, 1};
  };


  CasadiUnicycleMPC(bool verbose=false, casadi::Dict iopt_config={ 
    { "max_iter", 2000 }, 
    { "acceptable_tol", 1e-8 } ,
    {"acceptable_obj_change_tol", 1e-6}
  });

  std::map<std::string, casadi::DM> solve(const Config& mpcConf);


private:
  casadi::Function solve_mpc;
  std::map<std::string, casadi::DM> arg_;

};

std::vector<double> tf_to_global(const lgmath::se3::Transformation& T);
tactic::EdgeTransform tf_from_global(double x, double y, double theta);


struct PoseResultHomotopy
{
    std::vector<lgmath::se3::Transformation> poses;
    std::vector<double> barrier_q_max;
    std::vector<double> barrier_q_min;
};

struct CurvatureInfo
{
  Eigen::Vector3d center;
  double radius;

  inline double curvature() const {
    return 1 / radius;
  }

  static CurvatureInfo fromTransform(const lgmath::se3::Transformation &T_12);
};


// Declaring helper functions

// Helper function for generating reference measurements poses from a discrete path to use for tracking the path at a desired forward velocity
PoseResultHomotopy generateHomotopyReference(const std::vector<lgmath::se3::Transformation> &rolled_out_poses, tactic::LocalizationChain::Ptr);
PoseResultHomotopy generateHomotopyReference(const std::vector<double>& rolled_out_p, tactic::LocalizationChain::Ptr chain);

using Segment = std::pair<unsigned, unsigned>;
Segment findClosestSegment(const lgmath::se3::Transformation& T_wr, const tactic::LocalizationChain::Ptr chain, unsigned sid_start=0);
Segment findClosestSegment(const double p, const tactic::LocalizationChain::Ptr chain, unsigned sid_start=0);

lgmath::se3::Transformation interpolatePath(const lgmath::se3::Transformation& T_wr,
                const lgmath::se3::Transformation& seq_start, const lgmath::se3::Transformation& seq_end,
                 double& interp);
double findRobotP(const lgmath::se3::Transformation& T_wr, tactic::LocalizationChain::Ptr chain);

} //namespace vtr::path_planning