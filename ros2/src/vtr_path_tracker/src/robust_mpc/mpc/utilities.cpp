#include <vtr_path_tracker/robust_mpc/mpc/utilities.h>

namespace vtr {
namespace path_tracker {
namespace utils {

float thetaWrap(float th_in) {
  float th_out = th_in;
  if (th_in > M_PI) {
    th_out = th_in - 2 * M_PI;
  } else if (th_in < -M_PI) {
    th_out = th_in + 2 * M_PI;
  }
  return th_out;
}

float getSign(float number) {
  return (float) ((0.0 < number) - (number < 0.0));
}

} // utils
} // path_tracker
} // asrl
