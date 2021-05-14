#pragma once

#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

/**
 * \brief
 * \details
 * requires:
 *   qdata.[live_id]
 *   mdata.[map_id, T_q_m_prior, *recommended_experience]
 * outputs:
 *   qdata.[]
 *   mdata.[localization_map, localization_status]
 */
class SubMapExtractionModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "sub_map_extraction";

  /** \brief Collection of config parameters */
  struct Config {
    /** \brief Minimum depth of the sub map, in the temporal direction */
    int temporal_min_depth;

    /** \brief Maximum depth of the submap in the temporal direction */
    int temporal_max_depth;

    /** \brief Whether or not to search in the spatial direction. */
    bool search_spatially;

    /** \brief Search temporally for vertices within sigma_scale*sigma */
    double sigma_scale;

    /** \brief The angle weight to apply to the depth */
    double angle_weight;
  };

  SubMapExtractionModule(std::string name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  ~SubMapExtractionModule() = default;

  /**
   * \brief Sets the module's configuration.
   * \param config the input configuration.
   */
  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  /**
   * \brief Given a target vertex, this module will build a subgraph centered
   * on this vertex containing neighboring vertices up to a temporal depth, and
   * spatial neighbors not in the live run.
   * \param qdata The query data.
   * \param mdata The map data.
   * \param graph The Spatio Temporal Pose Graph.
   */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief Update the graph with the frame data for the live vertex */
  void updateGraphImpl(QueryCache &, MapCache &, const Graph::Ptr &,
                       VertexId) override {}

  /**
   * \brief Extract submap with a temporal depth and optional spatial neighbours
   * \param graph The spatio temporal pose graph
   * \param root The vertex id we're expanding from for the submap
   * \param current_run the live run that we don't want in the submap
   * \param mask TODO
   * \param temporal_depth TODO
   * \param spatial_neighbours TODO
   */
  static pose_graph::RCGraphBase::Ptr extractSubmap(
      const Graph &graph, const VertexId &root, uint32_t current_run,
      RunIdSet *mask, int temporal_depth, bool spatial_neighbours);

 private:
  /**
   * \brief calculates the depth of the localization map given the current
   * lateral uncertainty.
   * \param root The vertex the map is centered on.
   * \param lateral_uncertainty The 1-Sigma uncertainty in the x-axis.
   * \param graph The graph.
   */
  int calculateDepth(VertexId root, double lateral_uncertainty,
                     const std::shared_ptr<const Graph> &graph);

  /** \brief Algorithm Configuration */
  std::shared_ptr<Config> config_;
};

}  // namespace tactic
}  // namespace vtr
