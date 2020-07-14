#pragma once

#include <vtr/navigation/modules/base_module.h>

#include <asrl/messages/exp_recog_status.pb.h>

namespace std {
/// Display the experience recognition status message in a readable way
std::ostream& operator<<(std::ostream& os,
                         const asrl::status_msgs::ExpRecogStatus& msg);
}  // namespace std

namespace vtr {
namespace navigation {

/** Given a subgraph, return the run ids present. */
RunIdSet getRunIds(const asrl::pose_graph::RCGraphBase& subgraph);

/** Given a set of run ids, return those that are privileged (manual) runs.
 *
 * \param graph The graph (has manual info)
 * \param rids The set of run ids to check
 */
RunIdSet privelegedRuns(const asrl::pose_graph::RCGraphBase& graph,
                        RunIdSet rids);

/// Given a subgraph, keep only vertices from runs in the mask
asrl::pose_graph::RCGraphBase::Ptr maskSubgraph(
    const asrl::pose_graph::RCGraphBase::Ptr& graph, const RunIdSet& mask);

/** Fills up an experience recommendation set with n recommendations
 *
 * \param recommends (optional) recommendation to fill
 * \param distance_rids run similarity scores
 * \param n The max number of runs in the recommendation
 * \return the newly inserted runs
 */
RunIdSet fillRecommends(RunIdSet* recommends, const ScoredRids& distance_rids,
                        unsigned n);

/// Mask the localization subgraph based on upstream experience (run)
/// recommendations
class ExperienceTriageModule : public BaseModule {
 public:
  template <class T>
  using Ptr = std::shared_ptr<T>;

  /// Module name
  static constexpr auto type_str_ = "experience_triage";

  /// Module Configuration
  struct Config {
    bool in_the_loop = true;
    bool verbose = false;
    bool always_privileged = true;
  };

  ExperienceTriageModule(std::string name = type_str_) : BaseModule{name} {}

  /** Masks the localization map by the experience mask generated from upstream
   * recommenders.
   *
   * \param qdata Information about the live image.
   * \param mdata Information about the map.
   * \param graph The spatio-temporal pose graph.
   */
  virtual void run(QueryCache& qdata, MapCache& mdata,
                   const std::shared_ptr<const Graph>& graph);

  /** Saves any necessary information to the pose graph.
   *
   * \param qdata Information about the live image.
   * \param mdata Information about the map.
   * \param graph The spatio-temporal pose graph.
   * \param live_id TODO
   */
  virtual void updateGraph(QueryCache& qdata, MapCache& mdata,
                           const std::shared_ptr<Graph>& graph,
                           VertexId live_id);

  /// Sets the module configuration.
  void setConfig(const Config& config) {  ///< The new config
    config_ = config;
  }

 private:
  /// The status message to save to the graph
  asrl::status_msgs::ExpRecogStatus status_msg_;

  /// The module configuration
  Config config_;
};

}  // namespace navigation
}  // namespace vtr
