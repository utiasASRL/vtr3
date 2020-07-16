////////////////////////////////////////////////////////////////////////////////
/// @brief Source file for the ASRL vision package
/// @details
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

// Internal
#include <vtr/vision/sensors/sensor_model_base.h>
#include "vtr/vision/sensors/register_svd.h"


namespace vtr {
namespace vision {

// Explicit instantiation of the SVD registrator
template
bool registerSVD<3>(const Eigen::Matrix<double,3,Eigen::Dynamic>& a,
                    const Eigen::Matrix<double,3,Eigen::Dynamic>& b,
                    Eigen::Matrix<double,4,4> * tf,
                    const Eigen::Array<double,1,Eigen::Dynamic>& weights,
                    bool scaling);
template
bool registerSVD<2>(const Eigen::Matrix<double,2,Eigen::Dynamic>& a,
                    const Eigen::Matrix<double,2,Eigen::Dynamic>& b,
                    Eigen::Matrix<double,3,3> * tf,
                    const Eigen::Array<double,1,Eigen::Dynamic>& weights,
                    bool scaling);

template class SensorModelBase<Eigen::Matrix3d>;
template class SensorModelBase<Eigen::Matrix4d>;

}  // namespace vision
}  // namespace vtr
