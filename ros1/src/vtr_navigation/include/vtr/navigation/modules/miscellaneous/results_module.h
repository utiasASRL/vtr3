#pragma once

#include <vtr/navigation/modules/base_module.h>

namespace vtr {
namespace navigation {

class ResultsModule : public BaseModule {
 public:
  /** \brief Static module identifier.
   *
   * \todo change this to static_name
   */
  static constexpr auto type_str_ = "results";

  /** \brief Collection of config parameters
   */
  struct Config {
    /** \brief Depth of the sub map, in the temporal direction
     */
    std::string directory;
    std::string filename;
  };

  ResultsModule(std::string name = type_str_) : BaseModule{name} {}

  ~ResultsModule() {}

  /** \brief Records results data to a csv file.
   *
   * \param qdata The query data.
   * \param mdata The map data.
   * \param graph The Spatio Temporal Pose Graph.
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph) {}

  /** \brief Update the graph with the frame data for the live vertex
   */
  virtual void updateGraph(QueryCache &qdata, MapCache &mdata,
                           const std::shared_ptr<Graph> &graph,
                           VertexId live_id) {}

  /** \brief Sets the module's configuration.
   *
   * \param config the input configuration.
   */
  // \todo (old) Insert data to a vertex stream "/results/localization"
  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  /** \brief Algorithm Configuration
   */
  std::shared_ptr<Config> config_;

  std::ofstream out_stream_;
};

}  // namespace navigation
}  // namespace vtr
