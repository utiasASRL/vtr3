#include <gtest/gtest.h>
#include <random>

#include <vtr_logging/logging.hpp>
#include "vtr_logging/logging_init.hpp"


#include "vtr_path_planning/mpc/casadi_path_planners.hpp"

using namespace ::testing;
using namespace vtr::path_planning;
using namespace vtr;

constexpr unsigned NUM_STEPS = 15;


using SE2Pose = std::tuple<double, double, double>;
using TwoSE2Poses = std::pair<SE2Pose, SE2Pose>;

class CasadiTests : public TestWithParam<TwoSE2Poses>  {
 public:
  CasadiTests(){}
  ~CasadiTests() override {}

 
  void SetUp() override {}

  void TearDown() override {}

};


TEST_P(CasadiTests, JointBicycleMPC) {

  const auto& [init_pose_l, init_pose_f] = GetParam();
  

  CasadiBicycleMPCJoint solver;
  CasadiBicycleMPCJoint::Config config;
  config.VF = 1.0;
  config.vel_max = {1.5, 0.5};
  config.vel_min = {0, -0.5};
  config.Q_lat = 10;
  config.Q_lon = 4.0;
  config.Q_th = 7;
  config.R1 = 0;
  config.R2 = 3;
  config.Acc_R1 = 4;
  config.Acc_R2 = 5;
  config.Q_dist = 10;
  config.wheelbase = 0.65;
  config.distance = 1.5;

  
  config.previous_vel = {0.0, 0.0};
  config.T0 = {std::get<0>(init_pose_l), std::get<1>(init_pose_l), std::get<2>(init_pose_l)};

  for(int i = 1; i < config.N+1; i++){
    config.reference_poses.push_back({std::get<0>(init_pose_l) + i*config.VF*config.DT, 0, 0});
    config.reference_poses.push_back({0, 0, 0});
  }

  config.previous_vel_follower = {0.0, 0.0};
  config.T0_follower = {std::get<0>(init_pose_f), std::get<1>(init_pose_f), std::get<2>(init_pose_f)};
  for(int i = 1; i < config.N+1; i++){
    config.follower_reference_poses.push_back({std::get<0>(init_pose_f) + i*config.VF*config.DT, 0, 0});
    config.follower_reference_poses.push_back({0, 0, 0});
  }

  auto mpc_res = solver.solve(config);
  // CLOG(DEBUG, "test") << "Solver status: " << 
  // CLOG(DEBUG, "test") << "Shape: (" << (int)mpc_res["pose"].rows() << ", " << (int)mpc_res["pose"].columns() << ")";

  // CLOG(DEBUG, "test") << "Leader poses:";
  for(int i = 0; i < mpc_res["pose"].columns(); i++) {
    const auto& pose_i = mpc_res["pose"](casadi::Slice(), i).get_elements();
    // CLOG(DEBUG, "test") << "" << pose_i[0] << " " << pose_i[1] << " " << pose_i[2];
  }

  // CLOG(DEBUG, "test") << "Follower poses:";
  for(int i = 0; i < mpc_res["pose_follower"].columns(); i++) {
    const auto& pose_i = mpc_res["pose_follower"](casadi::Slice(), i).get_elements();
    // CLOG(DEBUG, "test") << "" << pose_i[0] << " " << pose_i[1] << " " << pose_i[2];
  }

  
  
}

INSTANTIATE_TEST_SUITE_P(MPCSet, CasadiTests, 
    Values(std::make_pair(std::make_tuple(1.0, 0.0, 0.0), std::make_tuple(0.0, 0.0, 0.0)),
          std::make_pair(std::make_tuple(1.5, 0.0, 0.0), std::make_tuple(0.0, 0.0, 0.0)),
          std::make_pair(std::make_tuple(2.0, 0.0, 0.0), std::make_tuple(0.0, 0.0, 0.0))));



int main(int argc, char** argv) {
  logging::configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
