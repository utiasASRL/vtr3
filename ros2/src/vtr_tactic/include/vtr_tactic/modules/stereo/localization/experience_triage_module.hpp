#pragma once

#include <vtr_messages/msg/exp_recog_status.hpp>
#include <vtr_tactic/modules/base_module.hpp>

namespace std {
/// Display the experience recognition status message in a readable way
std::ostream &operator<<(std::ostream &os,
                         const vtr_messages::msg::ExpRecogStatus &msg);
}  // namespace std

namespace vtr {
namespace tactic {

/** \brief Given a subgraph, return the run ids present */
RunIdSet getRunIds(const pose_graph::RCGraphBase &subgraph);

/**
 * \brief Given a set of run ids, return those that are privileged (manual)
 * runs
 * \param graph The graph (has manual info)
 * \param rids The set of run ids to check
 */
RunIdSet privilegedRuns(const pose_graph::RCGraphBase &graph, RunIdSet rids);

/** \brief Given a subgraph, keep only vertices from runs in the mask */
pose_graph::RCGraphBase::Ptr maskSubgraph(
    const pose_graph::RCGraphBase::Ptr &graph, const RunIdSet &mask);

/**
 * \brief Fills up an experience recommendation set with n recommendations
 * \param recommends recommendation to fill
 * \param distance_rids run similarity scores
 * \param n the max number of runs in the recommendation
 * \return the newly inserted runs
 */
RunIdSet fillRecommends(RunIdSet *recommends, const ScoredRids &distance_rids,
                        unsigned n);

/**
 * \brief Mask the localization subgraph based on upstream experience (run)
 * recommendations
 * requires:
 *   qdata.[]
 *   mdata.[localization_map, *recommended_experiences, *localization_status]
 * outputs:
 *   mdata.[recommended_experiences, localization_status]
 */
class ExperienceTriageModule : public BaseModule {
 public:
  template <class T>
  using Ptr = std::shared_ptr<T>;

  /** \brief Static module identifier. */
  static constexpr auto static_name = "experience_triage";

  /** \brief Collection of config parameters */
  struct Config {
    bool in_the_loop = true;
    bool verbose = false;
    bool always_privileged = true;
  };

  ExperienceTriageModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()) {}

  /**
   * \brief Masks the localization map by the experience mask generated from
   * upstream recommenders
   * \param qdata Information about the live image
   * \param mdata Information about the map
   * \param graph The spatio-temporal pose graph
   */
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /**
   * \brief Saves the status message to the pose graph
   * \param qdata Information about the live image
   * \param mdata Information about the map
   * \param graph The spatio-temporal pose graph
   * \param live_id The live vertex id
   */
  void updateGraphImpl(QueryCache &qdata, MapCache &mdata,
                       const Graph::Ptr &graph, VertexId live_id) override;

  /** \brief Sets the module configuration. */
  void setConfig(std::shared_ptr<Config> &config) { config_ = config; }

 private:
  /** \brief The status message to save to the graph */
  vtr_messages::msg::ExpRecogStatus status_msg_;

  /** \brief The module configuration */
  std::shared_ptr<Config> config_;
};

}  // namespace tactic
}  // namespace vtr
