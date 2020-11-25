#pragma once

#include <vtr_navigation/assemblies/base_assembly.hpp>

namespace vtr {
namespace navigation {

/** \brief Assembly to run modules associated with the localization pipeline. */
class LocalizerAssembly : public BaseAssembly {
 public:
  static constexpr auto type_str_ = "loc";

  /** \brief there are no requirements on the assembly, so just return true. */
  bool verify() const { return true; }

  /**
   * \brief Localize the frame data against the map vertex using the (sub)graph
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph);

  /**
   * \brief Updates the graph after the modules have run.
   * In the case of the localizer assembly, this includes updating the landmarks
   * in the live run to include matches to landmarks in the map.
   */
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           const VertexId &live_id);

 private:

  void saveResults(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<Graph> &graph,
                   const VertexId &live_id);

};

}  // namespace navigation
}  // namespace vtr
