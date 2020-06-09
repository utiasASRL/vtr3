#pragma once

#include <vtr/navigation/caches.h>
#include <vtr/navigation/modules/base_module.h>
#include <vtr/navigation/types.h>

#include <asrl/common/logging.hpp>

namespace asrl {
namespace navigation {

class BaseAssembly {
 public:
  /** \brief An unique identifier for creating assemblies. Subclass should
   * overwrite this.
   */
  static constexpr auto type_str_ = "assembly";

  BaseAssembly(const std::string name = type_str_) : name_{name} {}

  /** \brief Get the identifier of the assembly instance at runtime.
   *
   * The identifier is the string passed to the BaseAssembly constructor.
   */
  const std::string& getName() const { return name_; };

  /** \brief Localize the frame data against the map vertex using the (sub)graph
   */
  virtual void run(QueryCache& qdata, MapCache& mdata,
                   const std::shared_ptr<const Graph>& graph);

  /** \brief Update the graph with the frame data for the live vertex
   */
  virtual void updateGraph(QueryCache& qdata, MapCache& mdata,
                           const std::shared_ptr<Graph>& graph,
                           const VertexId& live_id);

  /** \brief Verify that the assembly is valid (after creation)
   *
   * \return true on success
   */
  virtual bool verify() const = 0;

  /** \brief Append a module to this assembly
   */
  void add_module(std::shared_ptr<BaseModule> module_ptr) {
    modules_.push_back(module_ptr);
  }

 protected:
  /** \brief The modules that make up the assembly
   */
  std::vector<std::shared_ptr<BaseModule>> modules_;

 private:
  const std::string name_;
};

}  // namespace navigation
}  // namespace asrl
