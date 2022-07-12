#include "vtr_path_planning/cbit/cbit_old.hpp"

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

//using namespace cbit;

namespace vtr {
namespace path_planning {


CBIT::CBIT(const Config::ConstPtr& config,
                               const RobotState::Ptr& robot_state,
                               const Callback::Ptr& callback)
    : BasePathPlanner(config, robot_state, callback), config_(config) {
  const auto node = robot_state->node.ptr();
}

CBIT::~CBIT() { stop(); }

void CBIT::initializeRoute(RobotState& robot_state) {
  /// \todo reset any internal state
  CLOG(INFO, "path_planning.teb") << "Made it into CBIT for real this time";
  auto& chain = *robot_state.chain;
  CLOG(INFO, "path_planning.teb") << "Chain Localized?: " << chain.isLocalized();
  /*
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_p_g, T_p_i_vec, curr_sid] =
      chain_info;

  CLOG(INFO, "path_planning.teb") << "stamp " << stamp;
  CLOG(INFO, "path_planning.teb") << "w_p_r_in_r: " << w_p_r_in_r;
  CLOG(INFO, "path_planning.teb") << "T_p_r: " << T_p_r;
  CLOG(INFO, "path_planning.teb") << "T_w_p: " << T_w_p;
  CLOG(INFO, "path_planning.teb") << "T_p_g: " << T_p_g;
  CLOG(INFO, "path_planning.teb") << "T_p_i_vec: " << T_p_i_vec;
  */
}

auto CBIT::computeCommand(RobotState& robot_state) -> Command {
  auto& chain = *robot_state.chain;
  if (!chain.isLocalized()) {
    CLOG(WARNING, "path_planning.teb")
        << "Robot is not localized, command to stop the robot";
    return Command();
  }
    else {
    CLOG(INFO, "path_planning.teb") << "Robot is now localized and we can start doing things";
  }

  // retrieve info from the localization chain
  const auto chain_info = getChainInfo(robot_state);
  auto [stamp, w_p_r_in_r, T_p_r, T_w_p, T_p_g, curr_sid] =
      chain_info;

  //CLOG(INFO, "path_planning.teb") << "stamp " << stamp;
  //CLOG(INFO, "path_planning.teb") << "w_p_r_in_r: " << w_p_r_in_r;
  //CLOG(INFO, "path_planning.teb") << "T_p_r: " << T_p_r;
  //CLOG(INFO, "path_planning.teb") << "T_w_p: " << T_w_p;
  //CLOG(INFO, "path_planning.teb") << "T_p_g: " << T_p_g;
  //CLOG(INFO, "path_planning.teb") << "T_p_i_vec: " << T_p_i_vec;
  //Testing trying to grab all of the teach key frame transforms
  //CLOG(INFO, "path_planning.teb") << "Key Frame 1 " << chain.pose(0);
  //CLOG(INFO, "path_planning.teb") << "Key Frame 2 " << chain.pose(1);


  // for now just always return a stop command (I think empty?)
  return Command();
}

auto CBIT::getChainInfo(RobotState& robot_state) -> ChainInfo {
  auto& chain = *robot_state.chain;
  auto lock = chain.guard();
  const auto stamp = chain.leaf_stamp();
  const auto w_p_r_in_r = chain.leaf_velocity();
  const auto T_p_r = chain.T_leaf_trunk().inverse();
  const auto T_w_p = chain.T_start_trunk();
  const auto curr_sid = chain.trunkSequenceId();
  auto target_sid = std::min(curr_sid + 1, (unsigned)(chain.size() - 1));

  const auto T_w_g = chain.pose(target_sid);
  const auto T_p_g = T_w_p.inverse() * T_w_g;

  return ChainInfo{stamp, w_p_r_in_r, T_p_r, T_w_p, T_p_g, curr_sid};
}



} // namespace path_planning
} // namespace vtr
