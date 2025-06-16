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
 * \file casadi_path_planners.hpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <casadi/casadi.hpp>
#include <vtr_path_planning/mpc/mpc_common.hpp>

namespace vtr::path_planning {

class CasadiMPC {
  public:
    PTR_TYPEDEFS(CasadiMPC);

    struct Config {
      PTR_TYPEDEFS(Config);

      virtual ~Config() {};  // for polymorphism
    };
    virtual ~CasadiMPC() {};
    virtual std::map<std::string, casadi::DM> solve(const Config& mpcConf) = 0;
};

class CasadiUnicycleMPC : public CasadiMPC {
public:
  PTR_TYPEDEFS(CasadiUnicycleMPC);
  using DM = casadi::DM;

  struct Config : public CasadiMPC::Config {
    PTR_TYPEDEFS(Config);
    // These values are defined the python code and exported
    // TODO add an automatic way to keep the code in sync
    static constexpr int nStates = 3;
    static constexpr int nControl = 2;
    static constexpr double alpha = 0.2;
    static constexpr int N = 15;
    static constexpr double DT = 0.25;
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

  std::map<std::string, casadi::DM> solve(const CasadiMPC::Config& mpcConf);


private:
  casadi::Function solve_mpc;
  std::map<std::string, casadi::DM> arg_;

};


class CasadiAckermannMPC : public CasadiMPC {
public:
  PTR_TYPEDEFS(CasadiAckermannMPC);
  using DM = casadi::DM;

  struct Config : public CasadiMPC::Config {
    PTR_TYPEDEFS(Config);
    // These values are defined the python code and exported
    // TODO add an automatic way to keep the code in sync
    static constexpr int nStates = 3;
    static constexpr int nControl = 2;
    static constexpr double alpha = 0.0;
    static constexpr int N = 15;
    static constexpr double DT = 0.25;
    DM previous_vel{nControl, 1};
    DM T0{nStates, 1};
    std::vector<DM> reference_poses;
    std::vector<double> up_barrier_q;
    std::vector<double> low_barrier_q;
    double VF = 0.0;
    DM vel_max{nControl, 1};
    double turning_radius = 1; //m
  };


  CasadiAckermannMPC(bool verbose=false, casadi::Dict iopt_config={ 
    { "max_iter", 2000 }, 
    { "acceptable_tol", 1e-8 } ,
    {"acceptable_obj_change_tol", 1e-6}
  });

  std::map<std::string, casadi::DM> solve(const CasadiMPC::Config& mpcConf);


private:
  casadi::Function solve_mpc;
  std::map<std::string, casadi::DM> arg_;

};

class CasadiUnicycleMPCFollower : public CasadiMPC {
public:
  PTR_TYPEDEFS(CasadiUnicycleMPCFollower);
  using DM = casadi::DM;

  struct Config : public CasadiMPC::Config {
    PTR_TYPEDEFS(Config);
    // These values are defined the python code and exported
    // TODO add an automatic way to keep the code in sync
    static constexpr int nStates = 3;
    static constexpr int nControl = 2;
    static constexpr double alpha = 0.2;
    static constexpr int N = 15;
    static constexpr double DT = 0.25;
    DM previous_vel{nControl, 1};
    DM T0{nStates, 1};
    std::vector<DM> leader_reference_poses;
    std::vector<DM> follower_reference_poses;
    std::vector<double> up_barrier_q;
    std::vector<double> low_barrier_q;
    double VF = 0.0;
    double distance = 0.5;
    double distance_margin = 1.0;
    DM vel_max{nControl, 1};
  };


  CasadiUnicycleMPCFollower(bool verbose=false, casadi::Dict iopt_config={ 
    { "max_iter", 2000 }, 
    { "acceptable_tol", 1e-8 } ,
    {"acceptable_obj_change_tol", 1e-6}
  });

  std::map<std::string, casadi::DM> solve(const CasadiMPC::Config& mpcConf);


private:
  casadi::Function solve_mpc;
  std::map<std::string, casadi::DM> arg_;

};

class CasadiBicycleMPC : public CasadiMPC {
public:
  PTR_TYPEDEFS(CasadiBicycleMPC);
  using DM = casadi::DM;

  struct Config : public CasadiMPC::Config {
    PTR_TYPEDEFS(Config);
    // These values are defined the python code and exported
    // TODO add an automatic way to keep the code in sync
    static constexpr int nStates = 3;
    static constexpr int nControl = 2;
    static constexpr double alpha = 0.0;
    static constexpr int N = 20;
    static constexpr double DT = 0.25;
    DM previous_vel{nControl, 1};
    DM T0{nStates, 1};
    std::vector<DM> reference_poses;
    std::vector<double> up_barrier_q;
    std::vector<double> low_barrier_q;
    double VF = 0.0;
    DM vel_max{nControl, 1};
    // The below are passed to the Casadi solver as tunable parameters
    double Q_x = 0.0;
    double Q_y = 0.0;
    double Q_th = 0.0;
    double R1 = 0.0;
    double R2 = 0.0;
    double Acc_R1 = 0.0;
    double Acc_R2 = 0.0;
    double lin_acc_max = 1.0; // m/s^2
    double ang_acc_max = 1.0;
    double Q_f = 0.0;
  };


  CasadiBicycleMPC(bool verbose=false, casadi::Dict iopt_config={ 
    { "max_iter", 2000 }, 
    { "acceptable_tol", 1e-8 } ,
    {"acceptable_obj_change_tol", 1e-6}
  });

  std::map<std::string, casadi::DM> solve(const CasadiMPC::Config& mpcConf);


private:
  casadi::Function solve_mpc;
  std::map<std::string, casadi::DM> arg_;

};


class CasadiBicycleMPCFollower : public CasadiMPC {
public:
  PTR_TYPEDEFS(CasadiBicycleMPCFollower);
  using DM = casadi::DM;

  struct Config : public CasadiBicycleMPC::Config {
    PTR_TYPEDEFS(Config);
    
    std::vector<DM> leader_reference_poses;
    double distance = 0.5;
    double distance_margin = 1.0;
    double Q_dist = 1.0;
  };


  CasadiBicycleMPCFollower(bool verbose=false, casadi::Dict iopt_config={ 
    { "max_iter", 2000 }, 
    { "acceptable_tol", 1e-8 } ,
    {"acceptable_obj_change_tol", 1e-6}
  });

  std::map<std::string, casadi::DM> solve(const CasadiMPC::Config& mpcConf);


private:
  casadi::Function solve_mpc;
  std::map<std::string, casadi::DM> arg_;

};

} //namespace vtr::path_planning
