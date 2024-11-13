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
 * \file mpc_path_planner.cpp
 * \author Jordy Sehn, Autonomous Space Robotics Lab (ASRL)
 */

#include <ranges>

#include "vtr_path_planning/mpc/mpc_path_planner.hpp"



namespace vtr::path_planning {
// CasadiMPC::Config::~Config() {}

// CasadiMPC::~CasadiMPC() {}


CasadiUnicycleMPC::CasadiUnicycleMPC( bool verbose, casadi::Dict ipopt_opts){
  casadi::Dict opts;
  if (!verbose) { 
    opts["print_time"] = 0;
    ipopt_opts["print_level"] = 0;
  }
  opts["ipopt"] = ipopt_opts;
  solve_mpc = nlpsol("solver", "ipopt", "libsolve_unicycle_mpc.so", opts);
}

std::map<std::string, casadi::DM> CasadiUnicycleMPC::solve(const CasadiMPC::Config& baseMpcConf) {
  using namespace casadi;

  const auto& mpcConf = dynamic_cast<const CasadiUnicycleMPC::Config& >(baseMpcConf);

  std::map<std::string, DM> arg;

  arg["lbx"] = DM::zeros(mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 1);
  arg["ubx"] = DM::zeros(mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 1);
  arg["lbx"].set(-DM::inf(), true, Slice(0, mpcConf.nStates*(mpcConf.N+1)));
  arg["ubx"].set(DM::inf(), true, Slice(0, mpcConf.nStates*(mpcConf.N+1)));
  
  // Velocity constraints
  arg["ubx"].set(mpcConf.vel_max(Slice(0)), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 2));
  arg["ubx"].set(mpcConf.vel_max(Slice(1)), true, Slice(mpcConf.nStates*(mpcConf.N+1)+1, mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 2));
  arg["lbx"].set(-mpcConf.vel_max(Slice(0)), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 2));
  arg["lbx"].set(-mpcConf.vel_max(Slice(1)), true, Slice(mpcConf.nStates*(mpcConf.N+1)+1, mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 2));

  arg["lbg"] = DM::zeros(mpcConf.nStates*(mpcConf.N+1) + mpcConf.N, 1);
  arg["ubg"] = DM::zeros(mpcConf.nStates*(mpcConf.N+1) + mpcConf.N, 1);

  if (mpcConf.up_barrier_q.size() > 0 && mpcConf.low_barrier_q.size() > 0) {
    arg["ubg"].set(DM(mpcConf.up_barrier_q), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.N));
    arg["lbg"].set(DM(mpcConf.low_barrier_q), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.N));
  } else {
    arg["ubg"].set(DM::inf(), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.N));
    arg["lbg"].set(-DM::inf(), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.N));
  }
  arg["x0"] = reshape(repmat(mpcConf.T0, 1, mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1), 1);
  arg["x0"] = vertcat(arg["x0"], DM::zeros(mpcConf.nControl* mpcConf.N, 1));

  arg["p"] = mpcConf.T0;

  for(int i = 0; i < mpcConf.N; i++) {
      arg["p"] = vertcat(arg["p"],
          mpcConf.reference_poses.at(i));
  }
  arg["p"] = vertcat(arg["p"], mpcConf.previous_vel);

  auto res = solve_mpc(arg);
  auto stats = solve_mpc.stats();

  if(stats["success"].as_bool() == false) { 
    CLOG(WARNING, "mpc.solver") << "Casadi error: " << stats["return_status"];
    // throw std::logic_error("Casadi was unable to find a feasible solution. Barrier constraint likely violated");
  }
  
  std::map<std::string, DM> output;
  output["pose"] = reshape(res["x"](Slice(0, mpcConf.nStates * (mpcConf.N + 1))), mpcConf.nStates,  mpcConf.N + 1);
  output["vel"] = reshape(res["x"](Slice(mpcConf.nStates * (mpcConf.N + 1), mpcConf.nStates * (mpcConf.N + 1) + mpcConf.nControl * mpcConf.N)), mpcConf.nControl,  mpcConf.N);

  return output;
}

std::vector<double> tf_to_global(const lgmath::se3::Transformation& T) {
  auto aang = lgmath::so3::rot2vec(T.C_ba());
  return {T.r_ab_inb()[0], T.r_ab_inb()[1], aang[2]};
}


CasadiAckermannMPC::CasadiAckermannMPC( bool verbose, casadi::Dict ipopt_opts){
  casadi::Dict opts;
  if (!verbose) { 
    opts["print_time"] = 0;
    ipopt_opts["print_level"] = 0;
  }
  opts["ipopt"] = ipopt_opts;
  solve_mpc = nlpsol("solver", "ipopt", "libsolve_ackermann_mpc.so", opts);
}


std::map<std::string, casadi::DM> CasadiAckermannMPC::solve(const CasadiMPC::Config& baseMpcConf) {
  using namespace casadi;
  const auto& mpcConf = dynamic_cast<const CasadiAckermannMPC::Config& >(baseMpcConf);


  std::map<std::string, DM> arg;

  arg["lbx"] = DM::zeros(mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 1);
  arg["ubx"] = DM::zeros(mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 1);
  arg["lbx"].set(-DM::inf(), true, Slice(0, mpcConf.nStates*(mpcConf.N+1)));
  arg["ubx"].set(DM::inf(), true, Slice(0, mpcConf.nStates*(mpcConf.N+1)));
  
  // Velocity constraints
  arg["ubx"].set(mpcConf.vel_max(Slice(0)), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 2));
  arg["ubx"].set(mpcConf.vel_max(Slice(1)), true, Slice(mpcConf.nStates*(mpcConf.N+1)+1, mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 2));
  arg["lbx"].set(DM(0), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 2));
  arg["lbx"].set(-mpcConf.vel_max(Slice(1)), true, Slice(mpcConf.nStates*(mpcConf.N+1)+1, mpcConf.nStates*(mpcConf.N+1) + mpcConf.nControl*mpcConf.N, 2));

  arg["lbg"] = DM::zeros(mpcConf.nStates*(mpcConf.N+1) + 3 * mpcConf.N, 1);
  arg["ubg"] = DM::zeros(mpcConf.nStates*(mpcConf.N+1) + 3 * mpcConf.N, 1);

  arg["ubg"].set(DM::inf(), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.N));
  arg["lbg"].set(-DM::inf(), true, Slice(mpcConf.nStates*(mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1) + mpcConf.N));
  

  arg["lbg"].set(DM(0), true, Slice(mpcConf.nStates*(mpcConf.N+1) + mpcConf.N, mpcConf.nStates*(mpcConf.N+1) + 3*mpcConf.N, 2));
  arg["lbg"].set(-DM::inf(), true, Slice(mpcConf.nStates*(mpcConf.N+1) + mpcConf.N + 1, mpcConf.nStates*(mpcConf.N+1) + 3*mpcConf.N, 2));

  arg["ubg"].set(DM::inf(), true, Slice(mpcConf.nStates*(mpcConf.N+1) + mpcConf.N, mpcConf.nStates*(mpcConf.N+1) + 3*mpcConf.N, 2));
  arg["ubg"].set(DM(0), true, Slice(mpcConf.nStates*(mpcConf.N+1) + mpcConf.N + 1, mpcConf.nStates*(mpcConf.N+1) + 3*mpcConf.N, 2));


  arg["x0"] = reshape(repmat(mpcConf.T0, 1, mpcConf.N+1), mpcConf.nStates*(mpcConf.N+1), 1);
  arg["x0"] = vertcat(arg["x0"], DM::zeros(mpcConf.nControl* mpcConf.N, 1));



  arg["p"] = mpcConf.T0;

  for(int i = 0; i < mpcConf.N; i++) {
      arg["p"] = vertcat(arg["p"],
          mpcConf.reference_poses.at(i));
  }
  arg["p"] = vertcat(arg["p"], mpcConf.previous_vel);
  arg["p"] = vertcat(arg["p"], DM(mpcConf.turning_radius));

  auto res = solve_mpc(arg);
  auto stats = solve_mpc.stats();

  if(stats["success"].as_bool() == false) { 
    CLOG(WARNING, "mpc.solver") << "Casadi error: " << stats["return_status"];
    throw std::logic_error("Casadi was unable to find a feasible solution. Barrier constraint likely violated");
  }
  
  std::map<std::string, DM> output;
  output["pose"] = reshape(res["x"](Slice(0, mpcConf.nStates * (mpcConf.N + 1))), mpcConf.nStates,  mpcConf.N + 1);
  output["vel"] = reshape(res["x"](Slice(mpcConf.nStates * (mpcConf.N + 1), mpcConf.nStates * (mpcConf.N + 1) + mpcConf.nControl * mpcConf.N)), mpcConf.nControl,  mpcConf.N);

  return output;
}


} //namespace vtr::path_planning