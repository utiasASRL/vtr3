#include "vtr_path_planning/cbit/cbit.hpp"

//using namespace cbit;

namespace vtr {
namespace path_planning {

void CBIT::initializeRouteTest(RobotState& robot_state) {
  /// \todo reset any internal state
  CLOG(INFO, "path_planning.teb") << "Made it into CBIT";
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

} // namespace path_planning
} // namespace vtr
