#pragma once

#include <asrl/navigation/caches.h>
#include <asrl/navigation/modules/base_module.h>
#include <asrl/navigation/types.h>
#include <asrl/common/logging.hpp>

namespace asrl {
namespace navigation {

class BaseAssembly {
 public:
  /** An unique identifier. Subclass should overwrite this.
   */
  static constexpr auto type_str_ = "assembly";

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
  /** \brief Only an assembly builder can create an assembly
   */
  BaseAssembly() {}

  /** \brief The modules that make up the assembly
   */
  std::vector<std::shared_ptr<BaseModule>> modules_;

 private:
};

}  // namespace navigation
}  // namespace asrl
