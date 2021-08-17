/**
 * \file utilities.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <vtr_path_tracker/robust_mpc/optimization/mpc_nominal_model.hpp>

namespace vtr {
namespace path_tracker {
namespace utils {

/**
 * \brief thetaWrap Wrap an angle to (-pi, pi]
 * \param th_in The angle
 * \note  Assumes abs(th_in) < 3pi
 * \return The equivalent angle between (-pi, pi]
 */
double thetaWrap(double th_in);

/** \brief Returns the sign of a number as a float (-1.,1.) */
double getSign(double number);

}  // namespace utils
}  // namespace path_tracker
}  // namespace vtr
