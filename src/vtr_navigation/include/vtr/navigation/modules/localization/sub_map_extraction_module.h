#pragma once

#include <vtr/navigation/modules/base_module.h>

namespace vtr {
namespace navigation {

/** \brief \todo
 */
class SubMapExtractionModule : public BaseModule {
 public:
  /** \brief Static module identifier.
   *
   * \todo change this to static_name
   */
  static constexpr auto type_str_ = "submap_extraction";

  /** \brief Collection of config parameters
   */
  struct Config {
    /// Minimum depth of the sub map, in the temporal direction
    int temporal_min_depth;

    /// Maximum depth of the submap in the temporal direction
    int temporal_max_depth;

    /// Whether or not to search in the spatial direction.
    bool search_spatially;

    /// Search temporally for vertices within sigma_scale*sigma
    double sigma_scale;

    /// The angle weight to apply to the depth
    double angle_weight;
  };

  SubMapExtractionModule(std::string name = type_str_) : BaseModule{name} {}

  ~SubMapExtractionModule() = default;

  /** \brief Given a target vertex, this module will build a subgraph centered
   * on this vertex containing neighboring vertices up to a temporal depth, and
   * spatial neighbors not in the live run.
   *
   * \param qdata The query data.
   * \param mdata The map data.
   * \param graph The Spatio Temporal Pose Graph.
   */
  virtual void run(QueryCache &qdata, MapCache &mdata,
                   const std::shared_ptr<const Graph> &graph);

  /** \brief Update the graph with the frame data for the live vertex
   */
  virtual void updateGraph(QueryCache &, MapCache &,
                           const std::shared_ptr<Graph> &, VertexId) {}

  /** \brief Sets the module's configuration.
   *
   * \param config the input configuration.
   */
  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

  /** \brief Extract submap with a temporal depth and optional spatial
   * neighbours
   *
   * \param graph The spatio temporal pose graph
   * \param root The vertex id we're expanding from for the submap
   * \param current_run the live run that we don't want in the submap
   * \param mask TODO
   * \param temporal_depth TODO
   * \param spatial_neighbours TODO
   */
  static asrl::pose_graph::RCGraphBase::Ptr extractSubmap(
      const Graph &graph, const VertexId &root, uint32_t current_run,
      RunIdSet *mask, int temporal_depth, bool spatial_neighbours);

 private:
  /** \brief calculates the depth of the localization map given the current
   * lateral uncertainty.
   *
   * \param root The vertex the map is centered on.
   * \param lateral_uncertainty The 1-Sigma uncertainty in the x-axis.
   * \param graph The graph.
   */
  int calculateDepth(VertexId root, double lateral_uncertainty,
                     const std::shared_ptr<const Graph> &graph);

  /** \brief Algorithm Configuration
   */
  std::shared_ptr<Config> config_;
};

}  // namespace navigation
}  // namespace vtr
