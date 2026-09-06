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
 * \file sparrow_explorer.cpp
 * \brief Knowledge-gradient KM exploration (C++ port of pomcp/exploration.py).
 */
#include "vtr_navigation/sparrow_explorer.hpp"

#include "vtr_navigation/survival_model.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <sstream>

namespace vtr {
namespace navigation {
namespace sparrow {

namespace {

double phi(double z) {
  return std::exp(-0.5 * z * z) / std::sqrt(2.0 * M_PI);
}

double Phi(double z) { return 0.5 * (1.0 + std::erf(z / std::sqrt(2.0))); }

/// f(-a) = phi(a) - a * Phi(-a) for a >= 0 (two-alternative KG factor).
double kg_factor(double a) {
  a = std::abs(a);
  return phi(a) - a * Phi(-a);
}

}  // namespace

// ---------------------------------------------------------------------------
// Graph costs
// ---------------------------------------------------------------------------

double KmExplorer::edgeCost(SVertex u, SVertex v) const {
  // Expected time to cross an edge, including the wait it is likely to
  // impose ("inflated") or bare travel time ("free"). See exploration.py.
  const double base = ctx_->edge_time(u, v);
  if (cfg_.detour_cost != "inflated" || model_ == nullptr) return base;
  const SEdge e = canonical_edge(u, v);
  auto lit = live_inflation_.find(e);
  if (lit != live_inflation_.end()) return base + lit->second;
  // On the robot every teach edge is blockable (the sim's context.blockable).
  return base + steady_inflation_;
}

double KmExplorer::dijkstraToGoal(SVertex source, const SEdge* forbidden) const {
  // Obstacle-free time source -> goal, optionally forbidding one edge.
  const SVertex goal = ctx_->goal;
  const SEdge banned =
      (forbidden != nullptr) ? *forbidden : SEdge{UINT64_MAX, UINT64_MAX};
  std::map<SVertex, double> dist;
  using QItem = std::pair<double, SVertex>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> heap;
  dist[source] = 0.0;
  heap.push({0.0, source});
  while (!heap.empty()) {
    auto [d, v] = heap.top();
    heap.pop();
    if (v == goal) return d;
    auto dit = dist.find(v);
    if (dit != dist.end() && d > dit->second + 1e-12) continue;
    auto nit = ctx_->neighbors.find(v);
    if (nit == ctx_->neighbors.end()) continue;
    for (const SVertex w : nit->second) {
      if (forbidden != nullptr && canonical_edge(v, w) == banned) continue;
      const double nd = d + edgeCost(v, w);
      auto it = dist.find(w);
      if (it == dist.end() || nd < it->second) {
        dist[w] = nd;
        heap.push({nd, w});
      }
    }
  }
  return kInf;
}

double KmExplorer::localDetourBetween(SVertex u, SVertex v,
                                      const SEdge& banned_in) const {
  const SEdge banned = banned_in;
  std::map<SVertex, double> dist;
  using QItem = std::pair<double, SVertex>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> heap;
  dist[u] = 0.0;
  heap.push({0.0, u});
  while (!heap.empty()) {
    auto [d, x] = heap.top();
    heap.pop();
    if (x == v) return d;
    auto dit = dist.find(x);
    if (dit != dist.end() && d > dit->second + 1e-12) continue;
    auto nit = ctx_->neighbors.find(x);
    if (nit == ctx_->neighbors.end()) continue;
    for (const SVertex w : nit->second) {
      if (canonical_edge(x, w) == banned) continue;
      const double nd = d + edgeCost(x, w);
      auto it = dist.find(w);
      if (it == dist.end() || nd < it->second) {
        dist[w] = nd;
        heap.push({nd, w});
      }
    }
  }
  return kInf;
}

double KmExplorer::meanLocalDetour() const {
  // Graph-typical surcharge for driving around one blocked edge. The sim
  // averages over context.blockable; on the robot every edge is blockable.
  std::vector<double> extras;
  extras.reserve(ctx_->edges.size());
  for (const auto& edge : ctx_->edges) {
    const double direct = edgeCost(edge.first, edge.second);
    const double around = localDetourBetween(edge.first, edge.second, edge);
    if (std::isfinite(around)) {
      extras.push_back(std::max(0.0, around - direct));
    } else {
      extras.push_back(detour_cap_);
    }
  }
  if (extras.empty()) return detour_cap_;
  double sum = 0.0;
  for (const double e : extras) sum += e;
  return std::min(detour_cap_, sum / static_cast<double>(extras.size()));
}

double KmExplorer::detourExtra(SVertex vertex, const SEdge& edge) const {
  // Surcharge of reaching the goal with `edge` forbidden, capped.
  double base;
  if (cfg_.detour_cost == "inflated" && model_ != nullptr) {
    // Both legs must use the same cost model or the difference is not a
    // surcharge; context.time_to_goal is obstacle-free by definition.
    base = dijkstraToGoal(vertex, nullptr);
  } else {
    auto it = ctx_->time_to_goal.find(vertex);
    base = (it != ctx_->time_to_goal.end()) ? it->second : kInf;
  }
  if (!std::isfinite(base)) return detour_cap_;
  const double alt = dijkstraToGoal(vertex, &edge);
  if (!std::isfinite(alt)) return detour_cap_;
  return std::min(detour_cap_, std::max(0.0, alt - base));
}

// ---------------------------------------------------------------------------
// Per-episode statistics
// ---------------------------------------------------------------------------

void KmExplorer::beginEpisode(const GraphContext* ctx,
                              const SparrowModel* model, int episode_idx,
                              int episodes_done, long encounters_seen) {
  ctx_ = ctx;
  model_ = model;
  episode_idx_ = episode_idx;
  detour_cap_ = (cfg_.w_max_s > 0.0) ? cfg_.w_max_s : 600.0;
  live_inflation_.clear();

  if (cfg_.detour_cost == "inflated") {
    // Steady-state expected wait per blockable edge: the chance a fresh
    // obstacle occupies it times how long it would take to clear,
    // marginalised over the class prior (same as the leaf evaluator).
    const double p_block = std::max(0.0, std::min(1.0, model->p_block));
    const double e_res = model->expected_residual_mixture(0.0);
    steady_inflation_ = std::max(0.0, p_block * e_res);
  } else {
    steady_inflation_ = 0.0;
  }

  // typical_detour: topology-only under "free" (cache per graph); under
  // "inflated" also a function of the frozen model (fold in a data count).
  size_t sig = ctx_->edges.size() * 1000003u + ctx_->neighbors.size() * 31u +
               static_cast<size_t>(ctx_->goal);
  if (cfg_.detour_cost == "inflated" && model->km != nullptr) {
    size_t n_km = 0;
    for (const auto& name : model->class_names)
      n_km += model->km->sampleCount(name);
    sig = sig * 1000003u + n_km * 97u +
          static_cast<size_t>(model->p_block * 1e6);
  }
  if (!td_valid_ || sig != td_signature_) {
    typical_detour_ = meanLocalDetour();
    td_signature_ = sig;
    td_valid_ = true;
  }

  const double lam_total =
      (episodes_done > 0)
          ? static_cast<double>(encounters_seen) / episodes_done
          : cfg_.encounter_rate_prior;
  // Model updates land after the episode, so only later episodes benefit.
  const int episodes_left = std::max(0, cfg_.num_episodes - episode_idx - 1);

  stats_.clear();
  for (size_t i = 0; i < model->class_names.size(); ++i) {
    const std::string& name = model->class_names[i];
    const double prob = model->class_probs[i];
    const double w_max_k = cfg_.w_max_s;
    const double sigma0 = std::max(1.0, 0.5 * w_max_k);

    // Uncensored KM samples for this class.
    std::vector<double> times;
    if (model->km != nullptr) {
      for (const auto& smp : model->km->getSamples(name)) {
        if (!smp.censored) times.push_back(smp.time);
      }
    }
    const int n = static_cast<int>(times.size());
    double s = sigma0;
    if (n >= 2) {
      double mean = 0.0;
      for (const double t : times) mean += t;
      mean /= n;
      double var = 0.0;
      for (const double t : times) var += (t - mean) * (t - mean);
      var /= (n - 1);  // ddof=1
      s = std::max(1.0, std::sqrt(var));
    }

    double sigma, sigma_tilde;
    if (cfg_.mean_uncertainty == "greenwood") {
      // Sampling variance of the KM restricted mean itself; censoring enters
      // through the at-risk counts. Falls back to the prior scale when the
      // class has no events yet.
      const double var = greenwoodRmstVariance(name, w_max_k);
      sigma = (var > 0.0) ? std::sqrt(var) : sigma0;
      // Greenwood variance falls off like 1/m; one further uncensored event
      // leaves sigma^2 * m/(m+1), and the predictive shift is the difference.
      sigma_tilde = sigma / std::sqrt(static_cast<double>(std::max(0, n)) + 1.0);
    } else {
      sigma = 1.0 / std::sqrt(1.0 / (sigma0 * sigma0) + n / (s * s));
      sigma_tilde = (sigma * sigma) / std::sqrt(sigma * sigma + s * s);
    }

    const double mu = model->expected_residual(name, 0.0);
    const double kg =
        sigma_tilde *
        kg_factor((mu - typical_detour_) / std::max(1e-9, sigma_tilde));
    const double lam_k = std::max(1e-6, lam_total * prob);
    double horizon;
    if (cfg_.value_mode == "acceleration") {
      // One forced sample buys a better model for the ~1/lam_k episodes
      // until one would have arrived anyway.
      horizon = 1.0 / lam_k;
    } else if (cfg_.value_mode == "capped") {
      horizon = std::min(static_cast<double>(episodes_left), cfg_.horizon_cap);
    } else {  // "horizon"
      horizon = static_cast<double>(episodes_left);
    }
    const double v_learn = cfg_.value_scale * horizon * lam_k * kg;
    stats_[name] = ClassStats{mu, sigma, s, kg, v_learn, n};
  }
}

void KmExplorer::setLiveBlockages(const std::vector<Candidate>& live) {
  // Price edges the robot currently knows are blocked by their conditional
  // expected residual E[R | age a]. No-op unless detour_cost == "inflated".
  live_inflation_.clear();
  if (cfg_.detour_cost != "inflated" || model_ == nullptr) return;
  for (const auto& [edge, age, label] : live) {
    double wait;
    if (!label.empty()) {
      wait = model_->expected_residual(label, age);
    } else {
      const auto post = model_->class_posterior_given_age(age);
      wait = 0.0;
      for (const auto& [k, pr] : post)
        wait += pr * model_->expected_residual(k, age);
    }
    live_inflation_[edge] = std::max(0.0, wait);
  }
}

// ---------------------------------------------------------------------------
// Encounter economics
// ---------------------------------------------------------------------------

std::pair<double, double> KmExplorer::eminAndP(const std::string& name,
                                               double age, double wait,
                                               int points) const {
  // E[min(R, W) | age, k] and P(R <= W | age, k) via trapezoid on S.
  if (wait <= 0.0) return {0.0, 0.0};
  double total = 0.0;
  double prev_u = 0.0;
  double prev_s = 1.0;
  double survival = 1.0;
  for (int i = 1; i <= points; ++i) {
    const double u = wait * i / points;
    survival = model_->residual_survival(name, u, age);
    total += 0.5 * (prev_s + survival) * (u - prev_u);
    prev_u = u;
    prev_s = survival;
  }
  return {total, std::max(0.0, 1.0 - survival)};
}

std::vector<double> KmExplorer::waitGrid(
    const std::map<std::string, double>& posterior, double w_cap) const {
  std::set<double> grid(cfg_.wait_durations.begin(),
                        cfg_.wait_durations.end());
  for (const auto& [name, prob] : posterior) {
    if (prob > 1e-3) grid.insert(cfg_.w_max_s);
  }
  std::vector<double> out;
  for (const double w : grid) {
    if (w > 0.0 && w <= w_cap) out.push_back(w);
  }
  return out;  // std::set is already sorted
}

double KmExplorer::greenwoodRmstVariance(const std::string& name,
                                         double w_max) const {
  // Var[RMST] = sum_j [int_{t_j}^{w_max} S(u) du]^2 * d_j / (Y_j (Y_j - d_j))
  // computed from the raw (time, censored) samples.
  if (model_ == nullptr || model_->km == nullptr) return 0.0;
  auto samples = model_->km->getSamples(name);
  if (samples.empty()) return 0.0;
  std::sort(samples.begin(), samples.end(),
            [](const SurvivalSample& a, const SurvivalSample& b) {
              return a.time < b.time;
            });
  // Distinct event (uncensored) times with counts and at-risk numbers.
  struct EventPoint {
    double t;
    int d;      // events at t
    int y;      // at risk just before t
    double s;   // S(t) after the drop
  };
  std::vector<EventPoint> events;
  const int n_total = static_cast<int>(samples.size());
  double s_curr = 1.0;
  size_t i = 0;
  while (i < samples.size()) {
    const double t = samples[i].time;
    int d = 0, c = 0;
    while (i < samples.size() && samples[i].time == t) {
      if (samples[i].censored) ++c; else ++d;
      ++i;
    }
    const int removed_before = static_cast<int>(i) - d - c;
    const int y = n_total - removed_before;
    if (d > 0 && y > 0) {
      s_curr *= (1.0 - static_cast<double>(d) / y);
      events.push_back({t, d, y, s_curr});
    }
  }
  if (events.empty()) return 0.0;
  // Integral of the step function S over [a, w_max].
  auto integral_S = [&](double a) -> double {
    if (a >= w_max) return 0.0;
    double total = 0.0, prev_t = a, s = 1.0;
    for (const auto& ev : events) {
      if (ev.t <= a) { s = ev.s; continue; }
      const double seg_end = std::min(ev.t, w_max);
      if (seg_end > prev_t) total += s * (seg_end - prev_t);
      if (ev.t >= w_max) return total;
      prev_t = ev.t;
      s = ev.s;
    }
    if (w_max > prev_t) total += s * (w_max - prev_t);
    return total;
  };
  double var = 0.0;
  for (const auto& ev : events) {
    if (ev.t > w_max) break;
    if (ev.y - ev.d <= 0) continue;
    const double integ = integral_S(ev.t);
    var += integ * integ * ev.d /
           (static_cast<double>(ev.y) * (ev.y - ev.d));
  }
  return var;
}

std::optional<KmExplorer::LearnDecision> KmExplorer::evaluateEdge(
    SVertex vertex, const SEdge& edge, double age, const std::string& label,
    double w_cap) const {
  if (model_ == nullptr || stats_.empty()) return std::nullopt;
  std::map<std::string, double> posterior;
  if (!label.empty()) {
    posterior[label] = 1.0;
  } else {
    posterior = model_->class_posterior_given_age(age);
  }
  std::vector<std::pair<std::string, double>> classes;
  for (const auto& [k, p] : posterior) {
    if (p > 1e-6 && stats_.count(k)) classes.push_back({k, p});
  }
  if (classes.empty()) return std::nullopt;

  const double detour = detourExtra(vertex, edge);
  const auto grid = waitGrid(posterior, w_cap);
  if (grid.empty()) return std::nullopt;

  std::vector<double> costs, values;
  costs.reserve(grid.size());
  values.reserve(grid.size());
  for (const double wait : grid) {
    double c = 0.0, v = 0.0;
    for (const auto& [name, prob] : classes) {
      const auto [emin, p_clear] = eminAndP(name, age, wait);
      c += prob * (emin + (1.0 - p_clear) * detour);
      // A censored observation still enters the KM fit; crediting it at zero
      // makes a longer wait the only way to raise the score.
      const double credit =
          p_clear + (1.0 - p_clear) * cfg_.censored_credit;
      v += prob * credit * stats_.at(name).v_learn;
    }
    costs.push_back(c);
    values.push_back(v);
  }
  const double j_exploit =
      std::min(detour, *std::min_element(costs.begin(), costs.end()));
  const double delta = label.empty() ? cfg_.delta_obs_exec_s : 0.0;

  int best_idx = -1;
  double best_score = 0.0;
  for (size_t gi = 0; gi < grid.size(); ++gi) {
    const double score = values[gi] - (delta + costs[gi] - j_exploit);
    if (score > best_score) {
      best_score = score;
      best_idx = static_cast<int>(gi);
    }
  }
  if (best_idx < 0) return std::nullopt;

  const double wait_star = grid[best_idx];
  std::ostringstream detail;
  detail << "W*=" << wait_star << "s score=" << best_score
         << "s D=" << detour << "s age=" << age << "s";
  for (const auto& [k, p] : classes) {
    const auto& st = stats_.at(k);
    detail << " " << k << ":p=" << p << ",V=" << st.v_learn
           << ",n=" << st.n_uncensored;
  }
  LearnDecision d;
  d.edge = edge;
  d.wait_s = wait_star;
  d.score_s = best_score;
  d.needs_observe = label.empty();
  d.detail = detail.str();
  return d;
}

std::optional<KmExplorer::LearnDecision> KmExplorer::propose(
    SVertex vertex, const std::vector<Candidate>& candidates,
    double w_cap) const {
  std::optional<LearnDecision> best;
  for (const auto& [edge, age, label] : candidates) {
    auto decision = evaluateEdge(vertex, edge, age, label, w_cap);
    if (decision.has_value() &&
        (!best.has_value() || decision->score_s > best->score_s)) {
      best = decision;
    }
  }
  return best;
}

}  // namespace sparrow
}  // namespace navigation
}  // namespace vtr
