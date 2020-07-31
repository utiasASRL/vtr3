/*
 * Author: Chris McKinnon
 * Email: chris.mckinnon@robotics.utias.utoronto.ca
 */

#include <vtr/path_tracker/robust_mpc/mpc/utilities.h>

// #include <tf/transform_datatypes.h>
// #include <asrl/common/rosutil/transformation_utilities.hpp>
namespace vtr {
namespace path_tracker {
namespace utils {

/**
 * @brief thetaWrap Wrap an angle to (-pi, pi]
 * @param th_in
 * @return
 */
float thetaWrap(float th_in)
{
  float th_out = th_in;
  if (th_in > M_PI){
    th_out = th_in - 2*M_PI;
  } else if (th_in < -M_PI) {
    th_out = th_in + 2*M_PI;
  }
  return th_out;
}


// Finds the sign of a number
float getSign(float number){
  return (float) ((0.0 < number) - (number < 0.0));
}

} // utils
} // path_tracker
} // asrl
