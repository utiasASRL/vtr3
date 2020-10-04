#pragma once

#include <vtr_navigation/assemblies/base_assembly.hpp>

namespace vtr {
namespace navigation {

/**
 * \brief Run on every new pair of caches pushed through the pipeline.
 * \details Pre-processes every image by converting colour spaces and extracting
 * features.
 */
class ConverterAssembly : public BaseAssembly {
 public:
  /** \brief An unique identifier for creating this assembly. */
  static constexpr auto type_str_ = "converter";

  ConverterAssembly() : BaseAssembly{type_str_} {}

  /**
   * \brief Call after creating the converter assembly.
   * \details \todo Should not this be included in the constructor?
   */
  bool verify() const {
    /// \todo Make sure that there are modules.
    return true;
  }

 private:
};

}  // namespace navigation
}  // namespace vtr
