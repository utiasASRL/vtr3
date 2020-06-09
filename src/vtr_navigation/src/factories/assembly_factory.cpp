#include <vtr/navigation/assemblies.h>
#include <vtr/navigation/factories/assembly_factory.h>

namespace vtr {
namespace navigation {

AssemblyFactory::assy_ptr AssemblyFactory::make() const {
  /// \todo maybe put this in constructor, because it should only be created
  /// once.
  FactoryTypeSwitch<assy_t> type_switch;
  type_switch.add<ConverterAssembly>();
  type_switch.add<QuickVoAssembly>();
  type_switch.add<RefinedVoAssembly>();
  // type_switch.add<LocalizerAssembly>();
  // type_switch.add<TerrainAssessmentAssembly>();

  LOG(INFO) << "making an assembly of type " << type_str_;
  auto assembly = type_switch.make(type_str_);
  if (!assembly) {
    auto msg = "unknown assembly of type " + type_str_;
    LOG(ERROR) << msg;
    throw std::invalid_argument(msg);
  }
  return assembly;
}
}  // namespace navigation
}  // namespace vtr
