#include <asrl/navigation/modules/base_module.h>
#include <opencv2/highgui/highgui.hpp>

namespace asrl {
namespace navigation {

std::mutex BaseModule::vis_mtx_;
}
}  // namespace asrl
