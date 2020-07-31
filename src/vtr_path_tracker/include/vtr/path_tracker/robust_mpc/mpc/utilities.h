/*
 * Author: Chris McKinnon
 * Email: chris.mckinnon@robotics.utias.utoronto.ca
 */

#pragma once

#include <vtr/path_tracker/robust_mpc/optimization/path_tracker_mpc_nominal_model.h>

namespace vtr {
namespace path_tracker {
namespace utils {

/**
 * @brief thetaWrap Wrap an angle to (-pi, pi]
 * @param th_in
 * @return
 */
float thetaWrap(float th_in);

/** Returns the sign of a number as a float (-1.,1.) */
float getSign(float number);

} // utils
} // path_tracker
} // asrl
