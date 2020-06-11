#include <vtr/navigation/modules/base_module.h>
#include <opencv2/highgui/highgui.hpp>

namespace vtr {
namespace navigation {

std::mutex BaseModule::vis_mtx_;
}
}  // namespace vtr
