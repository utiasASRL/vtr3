// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file sparrow_explorer.hpp
 * \brief Knowledge-gradient exploration of the learned clearance-time (KM)
 *        fits. Direct C++ port of vtr3_sim/pomcp/exploration.py.
 *
 * POMCP's search objective is the time-to-goal of the CURRENT episode.
 * Nothing in that objective rewards paying for an Observe and then waiting
 * long enough to collect an uncensored Kaplan-Meier sample, so over a long
 * deployment the fits either never form or stay coarse. This module prices
 * the missing term - the value a better fit has for FUTURE episodes - and,
 * when that value exceeds its immediate cost, commits the robot to a
 * learning macro: Observe (if the encounter is unlabeled) followed by
 * MaxWait(W*) with a wait budget chosen to maximise the net value of the
 * sample it will produce. The macro is committed - it overrides the search
 * until it completes - because commitment is what makes the priced sample
 * land instead of being censored by a detour.
 *
 * The math (see exploration.py for the full derivation):
 *   sigma_k^2     = 1 / (1/sigma0_k^2 + n_k / s_k^2)      (conjugate posterior)
 *   sigma~_k^2    = sigma_k^4 / (sigma_k^2 + s_k^2)       (predictive shift)
 *   KG_k          = sigma~_k * f(-|mu_k - D_bar| / sigma~_k),
 *                   f(z) = z Phi(z) + phi(z)               (two-alt. KG)
 *   V_k           = (N - j - 1) * lambda_k * KG_k          ("horizon" mode)
 *   p_k(W)        = P(R <= W | age a, k)
 *   c(W)          = sum_k pi_k [E[min(R,W) | a,k] + (1-p_k(W)) D]
 *   J_exploit     = min(D, min_W c(W))
 *   DeltaC(W)     = delta_obs * 1[unlabeled] + c(W) - J_exploit
 *   V(W)          = sum_k pi_k (p_k(W) + (1-p_k(W)) censored_credit) V_k
 *   commit iff max_W V(W) - DeltaC(W) > 0.
 *
 * The exploration decision is made at real decision points only; the tree
 * search itself is untouched.
 */
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "vtr_navigation/sparrow_planner.hpp"

namespace vtr {
namespace navigation {
namespace sparrow {

/// Knobs, mirroring sim POMCPExplorationConfig (defaults = main-table runs).
struct KgConfig {
  bool enabled = false;
  int num_episodes = 0;          ///< N: total missions the deployment runs
  double encounter_rate_prior = 1.0;
  std::string mean_uncertainty = "conjugate";  ///< "conjugate" | "greenwood"
  std::string detour_cost = "free";            ///< "free" | "inflated"
  std::string value_mode = "horizon";  ///< "horizon"|"acceleration"|"capped"
  double horizon_cap = 5.0;
  double value_scale = 1.0;
  double censored_credit = 0.0;
  double w_max_s = 300.0;        ///< W_max (single planning horizon, all types)
  double delta_obs_exec_s = 2.0; ///< execution charge of an Observe
  std::vector<double> wait_durations;
};

/// Ports exploration.KMExplorer: values one more KM sample and proposes
/// Observe + MaxWait(W*) macros.
class KmExplorer {
 public:
  /// Per-class learning value, frozen for one episode.
  struct ClassStats {
    double mu = 0.0;       ///< exploited mean residual (KM or fallback)
    double sigma = 0.0;    ///< posterior std of that mean
    double s = 0.0;        ///< one-sample noise
    double kg = 0.0;       ///< regret reduction per future decision
    double v_learn = 0.0;  ///< kg * expected remaining class-k encounters [s]
    int n_uncensored = 0;
  };

  /// A committed learning macro proposal.
  struct LearnDecision {
    SEdge edge{0, 0};
    double wait_s = 0.0;    ///< W* (0.0 while the class is still unknown)
    double score_s = 0.0;   ///< max_W V(W) - DeltaC(W) at decision time
    bool needs_observe = false;
    std::string detail;
  };

  /// (edge, age_s, label) - label empty when the encounter is unlabeled.
  using Candidate = std::tuple<SEdge, double, std::string>;

  explicit KmExplorer(KgConfig cfg) : cfg_(std::move(cfg)) {}

  /// Freeze per-class learning values against this episode's model.
  /// episode_idx is 0-based (sim semantics); episodes_done for lambda_total.
  void beginEpisode(const GraphContext* ctx, const SparrowModel* model,
                    int episode_idx, int episodes_done, long encounters_seen);

  /// Price edges currently known blocked by E[R | age] instead of the
  /// steady-state term (no-op unless detour_cost == "inflated").
  void setLiveBlockages(const std::vector<Candidate>& live);

  /// Best learning macro for one blocked edge, or nullopt if not worth it.
  std::optional<LearnDecision> evaluateEdge(SVertex vertex, const SEdge& edge,
                                            double age,
                                            const std::string& label,
                                            double w_cap) const;

  /// Highest-scoring learning macro among the candidate edges.
  std::optional<LearnDecision> propose(
      SVertex vertex, const std::vector<Candidate>& candidates,
      double w_cap) const;

  double maxWMax() const { return cfg_.w_max_s; }
  double typicalDetour() const { return typical_detour_; }
  const std::map<std::string, ClassStats>& stats() const { return stats_; }

 private:
  // -- graph costs (port of the _edge_cost/_dijkstra family) ------------------
  double edgeCost(SVertex u, SVertex v) const;
  double dijkstraToGoal(SVertex source, const SEdge* forbidden) const;
  double meanLocalDetour() const;
  double localDetourBetween(SVertex u, SVertex v, const SEdge& banned) const;
  double detourExtra(SVertex vertex, const SEdge& edge) const;

  // -- encounter economics ----------------------------------------------------
  /// E[min(R, W) | age, k] and P(R <= W | age, k) via trapezoid on S.
  std::pair<double, double> eminAndP(const std::string& name, double age,
                                     double wait, int points = 16) const;
  std::vector<double> waitGrid(const std::map<std::string, double>& posterior,
                               double w_cap) const;
  /// Greenwood-type sampling variance of the KM restricted mean (from raw
  /// samples; ports km.rmst_greenwood_variance).
  double greenwoodRmstVariance(const std::string& name, double w_max) const;

  KgConfig cfg_;
  const GraphContext* ctx_ = nullptr;
  const SparrowModel* model_ = nullptr;
  double detour_cap_ = 600.0;
  double steady_inflation_ = 0.0;
  std::map<SEdge, double> live_inflation_;
  double typical_detour_ = 600.0;
  std::map<std::string, ClassStats> stats_;
  int episode_idx_ = 0;
  // typical_detour cache: with detour_cost == "free" it is topology-only, so
  // recompute only when the graph signature changes; "inflated" also depends
  // on the frozen model, so a model signature is folded in.
  size_t td_signature_ = 0;
  bool td_valid_ = false;
};

}  // namespace sparrow
}  // namespace navigation
}  // namespace vtr
