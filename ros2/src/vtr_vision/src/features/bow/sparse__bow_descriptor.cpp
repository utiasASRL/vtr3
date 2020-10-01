#include <vtr_vision/features/bow/sparse_bow_descriptor.hpp>
#include <omp.h>
#include <numeric>
#include <algorithm>

namespace vtr {
namespace vision {

template class SparseBOWDescriptor<unsigned>;

}
}
