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
using DM = casadi::DM;

class CasadiMPC {
  public:
    PTR_TYPEDEFS(CasadiMPC);

    struct Config {
      // For our purposes, we will never need less than this to set up an MPC problem
      // Everything else should be set by the derived structs
      PTR_TYPEDEFS(Config);
      std::vector<casadi::DM> reference_poses;
      std::vector<double> up_barrier_q;
      std::vector<double> low_barrier_q;
      int nStates;
      int nControl;
      int N;
      double VF = 0.0;
      DM T0;
      double DT;
      DM vel_max;
      DM vel_min;
      DM previous_vel;
      std::vector<DM> cost_weights;
      int eop_index = -1; // index of the end of path in the reference poses

      Config(const int nStates = 3, const int nControl = 2, const int N = 15, const double DT = 0.25)
          : nStates(nStates), nControl(nControl), N(N), DT(DT) {
            T0 = DM::zeros(nStates, 1);
            vel_max = DM::zeros(nControl, 1);
            vel_min = DM::zeros(nControl, 1);
            previous_vel = DM::zeros(nControl, 1);
      };

      virtual ~Config() {}; 
    };
    virtual ~CasadiMPC() {};
    virtual std::map<std::string, casadi::DM> solve(const Config& mpcConf) = 0;
};

class CasadiUnicycleMPC : public CasadiMPC {
public:
  PTR_TYPEDEFS(CasadiUnicycleMPC);

  struct Config : public CasadiMPC::Config {
    PTR_TYPEDEFS(Config);
    double alpha = 0.0; // used for the unicycle model
    Config(const int nStates=3, const int nControl=2, const int N=15, const double DT=0.25)
        : CasadiMPC::Config(nStates, nControl, N, DT) {
    };
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

  struct Config : public CasadiMPC::Config {
    PTR_TYPEDEFS(Config);
    // These values are defined the python code and exported
    double turning_radius = 1; //m

    Config(const int nStates=3, const int nControl=2, const int N=15, const double DT=0.25)
        : CasadiMPC::Config(nStates, nControl, N, DT) {
    };
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

  struct Config : public CasadiMPC::Config {
    PTR_TYPEDEFS(Config);
    // These values are defined the python code and exported
    // TODO add an automatic way to keep the code in sync
    std::vector<DM> follower_reference_poses;
    std::vector<DM> leader_reference_poses;
    double distance = 0.5;
    double distance_margin = 1.0;
    Config(const int nStates=3, const int nControl=2, const int N=15, const double DT=0.25)
        : CasadiMPC::Config(nStates, nControl, N, DT) {
    };
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

  struct Config : public CasadiMPC::Config {
    PTR_TYPEDEFS(Config);
    // These values are defined the python code and exported
    // TODO add an automatic way to keep the code in sync
    double wheelbase = 0.55;
    // The below are passed to the Casadi solver as tunable parameters
    double Q_lat = 0.0;
    double Q_lon = 0.0;
    double Q_th = 0.0;
    double R1 = 0.0;
    double R2 = 0.0;
    double Acc_R1 = 0.0;
    double Acc_R2 = 0.0;
    double lin_acc_max = 1.0; // m/s^2
    double ang_acc_max = 1.0;
    double Q_f = 0.0;
    bool repeat_flipped = false;
    bool recovery = false; 


    Config(const int nStates=3, const int nControl=2, const int N=15, const double DT=0.25)
        : CasadiMPC::Config(nStates, nControl, N, DT) {
    };
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
  bool _reversing = false; 

};


class CasadiBicycleMPCFollower : public CasadiMPC {
public:
  PTR_TYPEDEFS(CasadiBicycleMPCFollower);
  

  struct Config : public CasadiBicycleMPC::Config {
    PTR_TYPEDEFS(Config);
    
    std::vector<DM> leader_reference_poses;
    double distance = 0.5;
    double distance_margin = 1.0;
    double Q_dist = 1.0;

    Config(const int nStates=3, const int nControl=2, const int N=15, const double DT=0.25)
        : CasadiBicycleMPC::Config(nStates, nControl, N, DT) {
    };
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
