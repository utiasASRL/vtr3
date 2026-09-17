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
 * \file sparrow_planner.cpp
 * \brief SPARROW (POMCP) planner core implementation. See sparrow_planner.hpp.
 */
#include "vtr_navigation/sparrow_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "vtr_navigation/survival_model.hpp"

namespace vtr {
namespace navigation {
namespace sparrow {

namespace {
constexpr double kEps = 1e-9;
}

// ===========================================================================
// SparrowModel (ports model.FrozenLearnedModel)
// ===========================================================================

double SparrowModel::survival(const std::string& obs_type, double t) const {
  if (!is_fitted(obs_type) || km == nullptr) {
    return std::exp(-std::max(0.0, t) / prior_residual_mean);
  }
  return km->survival(obs_type, std::max(0.0, t));
}

double SparrowModel::residual_survival(const std::string& obs_type, double u,
                                       double elapsed) const {
  if (!is_fitted(obs_type) || km == nullptr) {
    // Memoryless exponential prior.
    return std::exp(-std::max(0.0, u) / prior_residual_mean);
  }
  const double s_elapsed = km->survival(obs_type, std::max(0.0, elapsed));
  if (s_elapsed <= kEps) return 0.0;
  const double s_total =
      km->survival(obs_type, std::max(0.0, elapsed) + std::max(0.0, u));
  return std::min(1.0, std::max(0.0, s_total / s_elapsed));
}

double SparrowModel::expected_residual(const std::string& obs_type,
                                       double elapsed) const {
  if (!is_fitted(obs_type) || km == nullptr) return prior_residual_mean;
  // conditionalExpectedTime returns E[T | T > c] (total time).
  return std::max(
      0.0, km->conditionalExpectedTime(obs_type, std::max(0.0, elapsed)) -
               std::max(0.0, elapsed));
}

double SparrowModel::expected_residual_mixture(double elapsed) const {
  const auto posterior = class_posterior_given_age(elapsed);
  double total = 0.0;
  for (const auto& kv : posterior) {
    total += kv.second * expected_residual(kv.first, elapsed);
  }
  return total;
}

std::map<std::string, double> SparrowModel::class_posterior_given_age(
    double elapsed) const {
  std::map<std::string, double> out;
  double total = 0.0;
  for (size_t i = 0; i < class_names.size(); ++i) {
    const double w =
        std::max(0.0, class_probs[i] * survival(class_names[i], elapsed));
    out[class_names[i]] = w;
    total += w;
  }
  if (total <= 0.0) {
    const double p = class_names.empty() ? 1.0 : 1.0 / class_names.size();
    for (auto& kv : out) kv.second = p;
    return out;
  }
  for (auto& kv : out) kv.second /= total;
  return out;
}

std::string SparrowModel::sample_class(Rng& rng) const {
  if (class_names.empty()) return "unknown";
  const double u = rng.uniform();
  double acc = 0.0;
  for (size_t i = 0; i < class_names.size(); ++i) {
    acc += class_probs[i];
    if (u < acc) return class_names[i];
  }
  return class_names.back();
}

std::string SparrowModel::sample_class_given_age(Rng& rng,
                                                 double elapsed) const {
  if (class_names.empty()) return "unknown";
  const auto posterior = class_posterior_given_age(elapsed);
  const double u = rng.uniform();
  double acc = 0.0;
  for (const auto& name : class_names) {
    acc += posterior.at(name);
    if (u < acc) return name;
  }
  return class_names.back();
}

double SparrowModel::sample_residual(const std::string& obs_type, Rng& rng,
                                     double elapsed) const {
  if (!is_fitted(obs_type) || km == nullptr) {
    return rng.exponential(prior_residual_mean);
  }
  // Inverse-CDF bisection on the conditional survival
  //   F(r) = 1 - S(elapsed + r) / S(elapsed).
  const double u = std::min(1.0 - 1e-9, std::max(1e-9, rng.uniform()));
  // Find hi with cond_survival(hi) <= u (survival is non-increasing; the KM
  // exponential tail guarantees this terminates).
  double hi = 1.0;
  int guard = 0;
  while (residual_survival(obs_type, hi, elapsed) > u && guard++ < 40) {
    hi *= 2.0;
    if (hi > 1e6) break;
  }
  double lo = 0.0;
  for (int i = 0; i < 50; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (residual_survival(obs_type, mid, elapsed) > u) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

// ===========================================================================
// GraphContext (ports state.GraphContext)
// ===========================================================================

GraphContext::GraphContext(std::map<SVertex, std::vector<SVertex>> nbrs,
                           std::map<SEdge, double> tt, SVertex goal_v) {
  goal = goal_v;
  travel_time = std::move(tt);
  // Dedupe + sort neighbors.
  for (auto& kv : nbrs) {
    std::set<SVertex> uniq(kv.second.begin(), kv.second.end());
    neighbors[kv.first] = std::vector<SVertex>(uniq.begin(), uniq.end());
  }
  // Incident (canonical) edges per vertex + global edge list.
  std::set<SEdge> all_edges;
  for (const auto& kv : neighbors) {
    std::set<SEdge> inc;
    for (const auto& w : kv.second) {
      inc.insert(canonical_edge(kv.first, w));
    }
    incident_edges[kv.first] = std::vector<SEdge>(inc.begin(), inc.end());
    all_edges.insert(inc.begin(), inc.end());
  }
  edges = std::vector<SEdge>(all_edges.begin(), all_edges.end());

  // Obstacle-free Dijkstra to goal.
  using QItem = std::pair<double, SVertex>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> heap;
  time_to_goal[goal] = 0.0;
  heap.push({0.0, goal});
  while (!heap.empty()) {
    auto [d, v] = heap.top();
    heap.pop();
    auto it = time_to_goal.find(v);
    if (it != time_to_goal.end() && d > it->second + kEps) continue;
    auto nit = neighbors.find(v);
    if (nit == neighbors.end()) continue;
    for (const auto& w : nit->second) {
      const double nd = d + edge_time(w, v);
      auto wit = time_to_goal.find(w);
      if (wit == time_to_goal.end() || nd < wit->second - kEps) {
        time_to_goal[w] = nd;
        heap.push({nd, w});
      }
    }
  }
}

double GraphContext::edge_time(SVertex u, SVertex v) const {
  auto it = travel_time.find(canonical_edge(u, v));
  if (it == travel_time.end()) return kInf;
  return it->second;
}

double GraphContext::heuristic_cost_to_go(SVertex v) const {
  auto it = time_to_goal.find(v);
  return (it == time_to_goal.end()) ? kInf : it->second;
}

bool GraphContext::is_decision_vertex(SVertex v) const {
  if (v == goal) return true;
  auto it = neighbors.find(v);
  const size_t deg = (it == neighbors.end()) ? 0 : it->second.size();
  return deg != 2;
}

const std::vector<SVertex>& GraphContext::corridor(SVertex vertex,
                                                   SVertex first_hop) const {
  const auto key = std::make_pair(vertex, first_hop);
  auto cit = corridor_cache_.find(key);
  if (cit != corridor_cache_.end()) return cit->second;

  std::vector<SVertex> chain{first_hop};
  SVertex previous = vertex;
  SVertex current = first_hop;
  while (!is_decision_vertex(current)) {
    auto it = neighbors.find(current);
    if (it == neighbors.end()) break;
    std::vector<SVertex> options;
    for (const auto& w : it->second) {
      if (w != previous) options.push_back(w);
    }
    if (options.size() != 1) break;
    previous = current;
    current = options[0];
    chain.push_back(current);
    if (current == vertex) break;  // loop back: stop rather than cycle
  }
  return corridor_cache_.emplace(key, std::move(chain)).first->second;
}

// ===========================================================================
// ObstacleProcess (ports calendar.AbsoluteTimeObstacleProcess)
// ===========================================================================

ObstacleProcess::ObstacleProcess(const std::vector<SEdge>* edges,
                                 double spawn_rate_hz,
                                 const SparrowModel* model, Rng rng, double t0,
                                 bool no_adjacent_blocking)
    : edges_(edges),
      spawn_rate_hz_(spawn_rate_hz),
      model_(model),
      no_adjacent_blocking_(no_adjacent_blocking),
      rng_(rng),
      t_(t0) {
  next_spawn_time_ = draw_next_spawn(t_);
}

const ActiveObs* ObstacleProcess::obstacle_on(const SEdge& e) const {
  auto it = active_.find(e);
  return (it == active_.end()) ? nullptr : &it->second;
}

double ObstacleProcess::next_clearance_of(const SEdge& e) const {
  auto it = active_.find(e);
  return (it == active_.end()) ? kInf : it->second.t_clear;
}

double ObstacleProcess::draw_next_spawn(double t_from) {
  if (spawn_rate_hz_ <= 0.0 || edges_ == nullptr || edges_->empty()) {
    return kInf;
  }
  return t_from + rng_.exponential(1.0 / spawn_rate_hz_);
}

void ObstacleProcess::advance_to(double T) {
  if (T < t_ - kEps) return;  // never move backwards (defensive)
  while (true) {
    const double t_clear = pending_.empty() ? kInf : pending_.top().t;
    const double t_spawn = next_spawn_time_;
    const double t_next = (t_clear <= t_spawn) ? t_clear : t_spawn;
    if (t_next > T || t_next == kInf) break;
    if (t_clear <= t_spawn) {
      t_ = t_clear;
      process_clearance();
    } else {
      t_ = t_spawn;
      process_spawn(t_spawn);
      next_spawn_time_ = draw_next_spawn(t_spawn);
    }
  }
  t_ = T;
}

void ObstacleProcess::process_clearance() {
  const Pending top = pending_.top();
  pending_.pop();
  auto it = active_.find(top.edge);
  if (it == active_.end() || it->second.uid != top.uid) return;  // stale entry
  active_.erase(it);
}

void ObstacleProcess::process_spawn(double t_spawn) {
  if (edges_ == nullptr || edges_->empty() || model_ == nullptr) return;
  const SEdge edge = (*edges_)[rng_.integer(edges_->size())];
  if (active_.count(edge)) return;  // busy: drop spawn, keep Poisson clock
  if (no_adjacent_blocking_) {
    for (const auto& kv : active_) {
      const SEdge& a = kv.first;
      if (a.first == edge.first || a.first == edge.second ||
          a.second == edge.first || a.second == edge.second) {
        return;
      }
    }
  }
  const std::string obs_type = model_->sample_class(rng_);
  const double duration = model_->sample_duration(obs_type, rng_);
  if (!(duration > 0.0)) return;
  ActiveObs ob;
  ob.uid = next_uid_++;
  ob.obs_type = obs_type;
  ob.t_spawn = t_spawn;
  ob.t_clear = t_spawn + duration;
  active_[edge] = ob;
  pending_.push({ob.t_clear, ob.uid, edge});
}

const ActiveObs& ObstacleProcess::install_obstacle(const SEdge& e,
                                                   const std::string& type,
                                                   double t_spawn,
                                                   double t_clear, int uid) {
  ActiveObs ob;
  if (uid < 0) {
    ob.uid = next_uid_++;
  } else {
    ob.uid = uid;
    next_uid_ = std::max(next_uid_, uid + 1);
  }
  ob.obs_type = type;
  ob.t_spawn = t_spawn;
  ob.t_clear = t_clear;
  active_[e] = ob;
  pending_.push({ob.t_clear, ob.uid, e});
  return active_[e];
}

// ===========================================================================
// PlannerState
// ===========================================================================

bool PlannerState::is_classified(const SEdge& e) const {
  const ActiveObs* ob = process.obstacle_on(e);
  if (ob == nullptr) return false;
  auto it = classified.find(e);
  return it != classified.end() && it->second == ob->uid;
}

void PlannerState::record_sighting(const std::map<SEdge, bool>& statuses) {
  const double now = time();
  for (const auto& kv : statuses) {
    const SEdge& e = kv.first;
    if (!kv.second) {
      seen.erase(e);
      continue;
    }
    const ActiveObs* ob = process.obstacle_on(e);
    if (ob == nullptr) continue;  // defensive
    auto prior = seen.find(e);
    if (prior != seen.end() && prior->second.first == ob->uid) continue;
    seen[e] = {ob->uid, now};
  }
}

std::map<SEdge, bool> PlannerState::adjacent_statuses(
    const GraphContext& ctx) const {
  std::map<SEdge, bool> out;
  auto it = ctx.incident_edges.find(robot_vertex);
  if (it == ctx.incident_edges.end()) return out;
  for (const auto& e : it->second) out[e] = process.is_blocked(e);
  return out;
}

// ===========================================================================
// Actions (ports types.valid_actions)
// ===========================================================================

std::string SAction::str() const {
  std::ostringstream ss;
  switch (kind) {
    case TRAVERSE:
      ss << "Traverse(->" << first_hop << ")";
      break;
    case MAXWAIT:
      ss << "MaxWait(" << edge.first << "~" << edge.second << ", W=" << W
         << ")";
      break;
    case OBSERVE:
      ss << "Observe(" << edge.first << "~" << edge.second << ")";
      break;
  }
  return ss.str();
}

std::vector<SAction> valid_actions(SVertex vertex,
                                   const std::map<SEdge, bool>& statuses,
                                   const std::vector<SVertex>& neighbors,
                                   const std::set<SEdge>& classified_edges,
                                   const std::vector<double>& wait_set,
                                   bool allow_observe,
                                   const std::map<std::string,
                                                  std::vector<double>>*
                                       wait_set_by_class,
                                   const std::map<SEdge, std::string>*
                                       edge_classes) {
  std::vector<SAction> actions;
  // Traverse: only onto observed-free edges (sorted neighbor order).
  for (const auto& w : neighbors) {
    const SEdge ce = canonical_edge(vertex, w);
    auto it = statuses.find(ce);
    if (it == statuses.end()) continue;  // unobserved: unavailable
    if (!it->second) {
      SAction a;
      a.kind = SAction::TRAVERSE;
      a.edge = ce;
      a.first_hop = w;
      actions.push_back(a);
    }
  }
  // MaxWait + Observe on blocked adjacent edges.
  std::set<double> waits;
  for (const double w : wait_set) {
    if (w > 0.0) waits.insert(w);
  }
  for (const auto& w : neighbors) {
    const SEdge ce = canonical_edge(vertex, w);
    auto it = statuses.find(ce);
    if (it == statuses.end() || !it->second) continue;
    // A blockage the robot has PAID to Observe has a known clearance law, so
    // offer a grid resolved around THAT class's own mean rather than one grid
    // spanning every class - which is the point of having paid for the label.
    // Falls back to the shared grid for any class without its own.
    const std::set<double>* grid = &waits;
    std::set<double> class_waits;
    if (wait_set_by_class != nullptr && edge_classes != nullptr) {
      auto cit = edge_classes->find(ce);
      if (cit != edge_classes->end()) {
        auto git = wait_set_by_class->find(cit->second);
        if (git != wait_set_by_class->end()) {
          for (const double x : git->second) {
            if (x > 0.0) class_waits.insert(x);
          }
          if (!class_waits.empty()) grid = &class_waits;
        }
      }
    }
    for (const double W : *grid) {
      SAction a;
      a.kind = SAction::MAXWAIT;
      a.edge = ce;
      a.W = W;
      actions.push_back(a);
    }
    if (allow_observe && classified_edges.count(ce) == 0) {
      SAction a;
      a.kind = SAction::OBSERVE;
      a.edge = ce;
      actions.push_back(a);
    }
  }
  return actions;
}

// ===========================================================================
// TransitionModel (ports generator.TransitionModel)
// ===========================================================================

std::vector<SAction> TransitionModel::actions(PlannerState& s) const {
  settle(s);
  const auto statuses = s.adjacent_statuses(*ctx_);
  std::set<SEdge> classified;
  // Only edges the robot has PAID to Observe carry a class, and only those may
  // use a per-class wait grid (ports state.classified_adjacent_classes).
  std::map<SEdge, std::string> edge_classes;
  auto iit = ctx_->incident_edges.find(s.robot_vertex);
  if (iit != ctx_->incident_edges.end()) {
    for (const auto& e : iit->second) {
      if (!s.is_classified(e)) continue;
      classified.insert(e);
      if (!wait_set_by_class_.empty()) {
        const ActiveObs* ob = s.process.obstacle_on(e);
        if (ob != nullptr) edge_classes[e] = ob->obs_type;
      }
    }
  }
  auto nit = ctx_->neighbors.find(s.robot_vertex);
  static const std::vector<SVertex> kNoNeighbors;
  const auto& nbrs = (nit == ctx_->neighbors.end()) ? kNoNeighbors : nit->second;
  return valid_actions(s.robot_vertex, statuses, nbrs, classified, wait_set_,
                       allow_observe_,
                       wait_set_by_class_.empty() ? nullptr : &wait_set_by_class_,
                       edge_classes.empty() ? nullptr : &edge_classes);
}

StepResult TransitionModel::step(const PlannerState& s,
                                 const SAction& a) const {
  StepResult res;
  res.next = s;  // clone (value semantics)
  PlannerState& nxt = res.next;
  settle(nxt);

  double duration = 0.0;
  std::string class_result;

  if (a.kind == SAction::TRAVERSE) {
    const SVertex u = nxt.robot_vertex;
    std::vector<SVertex> chain;
    if (corridor_traversal_) {
      chain = ctx_->corridor(u, a.first_hop);
    } else {
      chain = {a.first_hop};
    }
    const double t0 = nxt.time();
    SVertex current = u;
    for (const auto& hop : chain) {
      // On-entry block check. In the Python sim the first edge is always
      // observed-free (Traverse is never offered onto a known-blocked edge);
      // on the robot a root Traverse may target an edge whose status was
      // UNKNOWN (out of sensor range), so the check applies to every hop.
      if (nxt.is_blocked(canonical_edge(current, hop))) break;
      const double leg = ctx_->edge_time(current, hop);
      nxt.process.advance_to(nxt.time() + leg);
      current = hop;
    }
    nxt.robot_vertex = current;
    duration = nxt.time() - t0;
  } else if (a.kind == SAction::MAXWAIT) {
    const ActiveObs* ob = nxt.process.obstacle_on(a.edge);
    const double t0 = nxt.time();
    const double deadline =
        (ob == nullptr) ? t0 + kEps : std::min(t0 + a.W, ob->t_clear);
    nxt.process.advance_to(std::max(deadline, t0 + kEps));
    duration = nxt.time() - t0;
  } else {  // OBSERVE
    const ActiveObs* ob = nxt.process.obstacle_on(a.edge);
    if (ob != nullptr) {
      class_result = ob->obs_type;  // perfect classifier
      nxt.mark_classified(a.edge, ob->uid);
    }
    nxt.process.advance_to(nxt.time() + delta_obs_);
    duration = delta_obs_;
  }

  const auto statuses = nxt.adjacent_statuses(*ctx_);
  nxt.record_sighting(statuses);

  res.obs.current_vertex = nxt.robot_vertex;
  res.obs.adjacent_statuses.assign(statuses.begin(), statuses.end());
  res.obs.duration_bin = static_cast<int64_t>(
      std::floor(std::max(0.0, duration) / std::max(1e-6, bin_width_)));
  res.obs.class_result = class_result;
  res.cost = std::max(kEps, duration);
  return res;
}

// ===========================================================================
// ParticleBelief (ports belief.ParticleBelief)
// ===========================================================================

void ParticleBelief::setEdgeWeights(const std::map<SEdge, double>& raw) {
  edge_p_.clear();
  edge_weight_.clear();
  if (raw.empty()) return;

  double total = 0.0;
  for (const auto& kv : raw) total += kv.second;
  const size_t n = raw.size();
  if (total <= 0.0 || n == 0) return;

  for (const auto& kv : raw) {
    // Absolute occupancy: p_block * w_e == 1 - (1 - p_micro)^n_micro.
    // Deliberately NOT normalised - see the header.
    edge_p_[kv.first] = kv.second;
    // Relative spawn preference only: mean-normalised.
    edge_weight_[kv.first] = kv.second * static_cast<double>(n) / total;
  }
}

double ParticleBelief::p_edge(const SEdge& edge) const {
  double p = std::max(0.0, std::min(1.0, model_->p_block));
  if (!edge_p_.empty()) {
    auto it = edge_p_.find(edge);
    if (it != edge_p_.end()) return std::min(0.95, p * it->second);
  }
  if (!edge_weight_.empty()) {
    auto it = edge_weight_.find(edge);
    const double w = (it == edge_weight_.end()) ? 1.0 : it->second;
    p = std::min(0.95, p * w);
  }
  return p;
}

double ParticleBelief::weight_edge(const SEdge& edge) const {
  if (edge_weight_.empty()) return 1.0;
  auto it = edge_weight_.find(edge);
  return (it == edge_weight_.end()) ? 1.0 : it->second;
}

void ParticleBelief::initialize(const LocalObservation& local) {
  particles_.clear();
  particles_.reserve(num_particles_);
  for (int i = 0; i < num_particles_; ++i) {
    particles_.push_back(new_particle(local));
  }
}

PlannerState ParticleBelief::new_particle(const LocalObservation& local) {
  Rng rng(derive_seed(seed_, ++stream_counter_));
  ObstacleProcess process(&ctx_->edges, model_->spawn_rate_hz, model_,
                          Rng(derive_seed(seed_, 0x100000 + stream_counter_)),
                          local.time, no_adj_);

  // Unobserved edges: memory-conditioned or steady-state Bernoulli(p_block).
  for (const auto& edge : ctx_->edges) {
    if (local.statuses.count(edge)) continue;  // observed: handled below
    auto mit = local.memory.find(edge);
    if (mit != local.memory.end()) {
      install_from_memory(process, local, edge, mit->second, rng);
      continue;
    }
    // Contracted planning: the composed occupancy of the whole corridor,
    // not the flat micro rate.
    const double p = p_edge(edge);
    if (rng.uniform() >= p) continue;
    const std::string obs_type = model_->sample_class(rng);
    const double residual = model_->sample_residual(obs_type, rng, 0.0);
    if (residual <= 0.0) continue;
    // Spawn instant of an unobserved obstacle is not identifiable.
    process.install_obstacle(edge, obs_type, local.time, local.time + residual);
  }

  PlannerState state;
  state.robot_vertex = local.vertex;
  state.process = std::move(process);

  // Observed statuses are enforced.
  for (const auto& kv : local.statuses) {
    if (kv.second == 1) {
      install_tracked_blockage(state, local, kv.first, rng);
    } else {
      state.process.remove_obstacle(kv.first);
    }
  }

  // Seed the simulated information set with what the robot has really seen.
  const double now = local.time;
  for (const auto& kv : local.statuses) {
    if (kv.second != 1) continue;
    const ActiveObs* ob = state.process.obstacle_on(kv.first);
    if (ob == nullptr) continue;
    double age = 0.0;
    auto ait = local.ages.find(kv.first);
    if (ait != local.ages.end()) age = std::max(0.0, ait->second);
    state.seen[kv.first] = {ob->uid, now - age};
  }
  for (const auto& kv : local.memory) {
    if (!kv.second.blocked || state.seen.count(kv.first)) continue;
    const ActiveObs* ob = state.process.obstacle_on(kv.first);
    if (ob == nullptr) continue;
    const double first_seen =
        kv.second.t_obs - std::max(0.0, kv.second.age_at_obs);
    state.seen[kv.first] = {ob->uid, first_seen};
  }
  return state;
}

void ParticleBelief::install_from_memory(ObstacleProcess& process,
                                         const LocalObservation& local,
                                         const SEdge& edge,
                                         const EdgeMemoryRec& mem, Rng& rng) {
  const double delta = std::max(0.0, local.time - mem.t_obs);
  if (mem.blocked) {
    const double age0 = std::max(0.0, mem.age_at_obs);
    std::map<std::string, double> posterior;
    if (!mem.label.empty()) {
      posterior[mem.label] = 1.0;
    } else {
      posterior = model_->class_posterior_given_age(age0);
    }
    // (class, weight = p * S(a0+delta)/S(a0)) pairs.
    std::vector<std::pair<std::string, double>> weights;
    double alive = 0.0;
    for (const auto& kv : posterior) {
      if (kv.second <= 0.0) continue;
      const double w =
          kv.second * model_->residual_survival(kv.first, delta, age0);
      weights.push_back({kv.first, w});
      alive += w;
    }
    if (!weights.empty() && rng.uniform() < alive) {
      const double u = rng.uniform() * alive;
      double acc = 0.0;
      std::string obs_type = weights.back().first;
      for (const auto& kv : weights) {
        acc += kv.second;
        if (u < acc) {
          obs_type = kv.first;
          break;
        }
      }
      const double residual =
          model_->sample_residual(obs_type, rng, age0 + delta);
      if (residual > 0.0) {
        process.install_obstacle(edge, obs_type,
                                 local.time - (age0 + delta),
                                 local.time + residual);
        return;
      }
    }
    // Remembered encounter cleared; fall through to fresh-spawn chance.
  }
  maybe_fresh_spawn(process, local, edge, delta, rng);
}

void ParticleBelief::maybe_fresh_spawn(ObstacleProcess& process,
                                       const LocalObservation& local,
                                       const SEdge& edge, double delta,
                                       Rng& rng) {
  const double p_block = p_edge(edge);
  if (p_block <= 0.0 || delta <= 0.0) return;
  const double rate_edge =
      weight_edge(edge) * model_->spawn_rate_hz / std::max(1, model_->num_edges);
  const double p_new =
      p_block * (1.0 - std::exp(-rate_edge * delta / p_block));
  if (rng.uniform() >= p_new) return;
  const std::string obs_type = model_->sample_class(rng);
  const double residual = model_->sample_residual(obs_type, rng, 0.0);
  if (residual <= 0.0) return;
  process.install_obstacle(edge, obs_type, local.time, local.time + residual);
}

void ParticleBelief::install_tracked_blockage(PlannerState& state,
                                              const LocalObservation& local,
                                              const SEdge& edge, Rng& rng) {
  double age = 0.0;
  auto ait = local.ages.find(edge);
  if (ait != local.ages.end()) age = std::max(0.0, ait->second);
  std::string label;
  auto lit = local.labels.find(edge);
  if (lit != local.labels.end()) label = lit->second;
  const double first_sight = local.time - age;

  double elapsed = age;
  auto mit = local.memory.find(edge);
  if (mit != local.memory.end() && mit->second.blocked &&
      mit->second.t_obs < first_sight - kEps) {
    // Re-sighting after a gap: mixture of "same obstacle survived the gap"
    // vs "cleared and a fresh one arrived".
    const EdgeMemoryRec& mem = mit->second;
    const double a1 = std::max(0.0, mem.age_at_obs);
    const double delta = local.time - mem.t_obs;
    std::string prior_label = !label.empty() ? label : mem.label;
    std::map<std::string, double> posterior;
    if (!prior_label.empty()) {
      posterior[prior_label] = 1.0;
    } else {
      posterior = model_->class_posterior_given_age(a1);
    }
    double w_same = 0.0;
    for (const auto& kv : posterior) {
      if (kv.second <= 0.0) continue;
      w_same += kv.second * model_->residual_survival(kv.first, delta, a1);
    }
    const double gap = std::max(0.0, first_sight - mem.t_obs);
    const double p_block = p_edge(edge);
    const double rate_edge =
        weight_edge(edge) * model_->spawn_rate_hz / std::max(1, model_->num_edges);
    const double p_new =
        (p_block > 0.0)
            ? p_block * (1.0 - std::exp(-rate_edge * gap / p_block))
            : 0.0;
    const double w_new = (1.0 - w_same) * p_new;
    const double total = w_same + w_new;
    if (total > 0.0 && rng.uniform() < w_same / total) {
      elapsed = a1 + delta;
    }
  }

  std::string obs_type;
  if (!label.empty()) {
    obs_type = label;
  } else {
    obs_type = model_->sample_class_given_age(rng, elapsed);
  }
  double residual = model_->sample_residual(obs_type, rng, elapsed);
  residual = std::max(residual, kEps);
  const ActiveObs& ob = state.process.install_obstacle(
      edge, obs_type, local.time - elapsed, local.time + residual);
  if (!label.empty()) {
    state.mark_classified(edge, ob.uid);
  } else {
    state.classified.erase(edge);
  }
}

// ===========================================================================
// SparrowSolver (ports solver.CostMinimizingPOMCP + ShortestPathRollout)
// ===========================================================================

double SparrowSolver::cutoff_cost(const PlannerState& state) const {
  const double h = transition_->context().heuristic_cost_to_go(
      state.robot_vertex);
  if (!std::isfinite(h)) return settings_.max_sim_time_s;
  return h * settings_.cutoff_congestion_factor;
}

PlanResult SparrowSolver::plan(const std::vector<PlannerState>& particles,
                               const std::vector<SAction>& root_actions) {
  const auto started = std::chrono::steady_clock::now();
  HistoryNode root;
  nodes_ = 1;
  max_depth_seen_ = 0;

  const double budget = settings_.max_planning_time_s;
  int simulations = 0;
  while (simulations < settings_.num_simulations) {
    if (budget > 0.0) {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        started)
              .count();
      if (elapsed >= budget) break;
    }
    if (particles.empty()) break;
    PlannerState state = particles[simulations % particles.size()];  // clone
    simulate(std::move(state), root, 0, particles[0].time(), &root_actions);
    ++simulations;
  }

  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
          .count();

  PlanResult result;
  result.simulations = simulations;
  result.planning_time_s = elapsed;
  result.tree_nodes = nodes_;
  result.tree_depth = max_depth_seen_;
  for (const auto& kv : root.children) {
    RootActionStat s;
    s.action = kv.first;
    s.visits = kv.second.visits;
    s.q_cost = kv.second.q_cost;
    result.root_actions.push_back(s);
  }
  std::sort(result.root_actions.begin(), result.root_actions.end(),
            [](const RootActionStat& a, const RootActionStat& b) {
              if (a.visits != b.visits) return a.visits > b.visits;
              return a.q_cost < b.q_cost;
            });
  // Robust child, but decided on COST among the actions that were explored
  // comparably: take the lowest mean cost among those with at least
  // robust_visit_frac of the top visit count. With a pure visit count an
  // action that UCB happened to concentrate on beats a better-scoring one it
  // merely sampled less (see robust_visit_frac).
  {
    int top_visits = 0;
    for (const auto& s : result.root_actions)
      top_visits = std::max(top_visits, s.visits);
    if (top_visits > 0) {
      const double frac = std::min(1.0, std::max(0.0,
                                                 settings_.robust_visit_frac));
      const int cutoff = static_cast<int>(std::ceil(frac * top_visits));
      double best_q = std::numeric_limits<double>::infinity();
      for (const auto& s : result.root_actions) {
        if (s.visits <= 0 || s.visits < cutoff) continue;
        if (s.q_cost < best_q) {
          best_q = s.q_cost;
          result.action = s.action;
        }
      }
      // Nothing cleared the bar (frac == 1.0 with no exact tie): fall back.
      if (!result.action.has_value()) {
        for (const auto& s : result.root_actions) {
          if (s.visits > 0) { result.action = s.action; break; }
        }
      }
    }
  }
  return result;
}

double SparrowSolver::simulate(PlannerState state, HistoryNode& node,
                               int depth, double sim_start_time,
                               const std::vector<SAction>* forced_actions) {
  if (transition_->is_terminal(state)) return 0.0;
  if (depth > max_depth_seen_) max_depth_seen_ = depth;
  if (depth >= settings_.max_depth) return cutoff_cost(state);
  if ((state.time() - sim_start_time) >= settings_.max_sim_time_s) {
    return cutoff_cost(state);
  }

  std::vector<SAction> actions;
  if (forced_actions != nullptr) {
    transition_->settle(state);
    actions = *forced_actions;
  } else {
    actions = transition_->actions(state);
  }
  if (actions.empty()) return cutoff_cost(state);

  if (!node.expanded) {
    for (const auto& a : actions) node.children.emplace(a, ActionNode{});
    node.expanded = true;
    node.visits += 1;
    return rollout(std::move(state), sim_start_time);
  }
  for (const auto& a : actions) {
    node.children.emplace(a, ActionNode{});
  }

  // forced_actions is non-null only for the root history node.
  const SAction* action =
      select(node, actions, /*is_root=*/forced_actions != nullptr);
  ActionNode& action_node = node.children.at(*action);

  StepResult sr = transition_->step(state, *action);
  auto cit = action_node.children.find(sr.obs);
  if (cit == action_node.children.end()) {
    cit = action_node.children
              .emplace(sr.obs, std::make_unique<HistoryNode>())
              .first;
    ++nodes_;
  }

  const double total =
      sr.cost + simulate(std::move(sr.next), *cit->second, depth + 1,
                         sim_start_time, nullptr);
  node.visits += 1;
  action_node.update(total);
  return total;
}

const SAction* SparrowSolver::select(HistoryNode& node,
                                     const std::vector<SAction>& actions,
                                     bool is_root) {
  for (const auto& a : actions) {
    if (node.children.at(a).visits == 0) return &a;
  }

  // Reserve an even floor of the budget across the root's actions before UCB
  // concentrates. See SparrowSearchSettings::root_explore_frac.
  if (is_root && settings_.root_explore_frac > 0.0 && !actions.empty()) {
    const double budget =
        settings_.num_simulations * settings_.root_explore_frac;
    if (settings_.root_explore_by_class) {
      std::map<int, int> members;  // SAction::kind -> how many share the class
      for (const auto& a : actions) members[static_cast<int>(a.kind)] += 1;
      const double per_class =
          budget / static_cast<double>(std::max<size_t>(1, members.size()));
      for (const auto& a : actions) {
        const int floor_visits = static_cast<int>(
            per_class / std::max(1, members[static_cast<int>(a.kind)]));
        if (floor_visits > 0 && node.children.at(a).visits < floor_visits) {
          return &a;
        }
      }
    } else {
      const int floor_visits =
          static_cast<int>(budget / static_cast<double>(actions.size()));
      if (floor_visits > 0) {
        for (const auto& a : actions) {
          if (node.children.at(a).visits < floor_visits) return &a;
        }
      }
    }
  }

  const double log_n = std::log(std::max(1, node.visits));
  const SAction* best = nullptr;
  double best_score = kInf;
  for (const auto& a : actions) {
    const ActionNode& child = node.children.at(a);
    const double score =
        child.q_cost - settings_.c_uct * std::sqrt(log_n / child.visits);
    if (score < best_score) {
      best_score = score;
      best = &a;
    }
  }
  return best != nullptr ? best : &actions[0];
}

SAction SparrowSolver::rollout_action(
    PlannerState& state, const std::vector<SAction>& actions) const {
  // ShortestPathRollout: greedy free hop toward goal; longest wait if boxed in.
  const GraphContext& ctx = transition_->context();
  const SAction* best_traverse = nullptr;
  double best_traverse_cost = kInf;
  const SAction* best_wait = nullptr;
  for (const auto& a : actions) {
    if (a.kind == SAction::TRAVERSE) {
      const double c = ctx.edge_time(state.robot_vertex, a.first_hop) +
                       ctx.heuristic_cost_to_go(a.first_hop);
      if (c < best_traverse_cost) {
        best_traverse_cost = c;
        best_traverse = &a;
      }
    } else if (a.kind == SAction::MAXWAIT) {
      if (best_wait == nullptr || a.W > best_wait->W) best_wait = &a;
    }
  }
  if (best_traverse != nullptr) return *best_traverse;
  if (best_wait != nullptr) return *best_wait;
  return actions[0];
}

double SparrowSolver::rollout(PlannerState state, double sim_start_time) {
  double total = 0.0;
  PlannerState current = std::move(state);
  while (true) {
    if (transition_->is_terminal(current)) return total;
    if ((current.time() - sim_start_time) >= settings_.max_sim_time_s) {
      return total + cutoff_cost(current);
    }
    const auto actions = transition_->actions(current);
    if (actions.empty()) return total + cutoff_cost(current);
    const SAction a = rollout_action(current, actions);
    StepResult sr = transition_->step(current, a);
    total += sr.cost;
    current = std::move(sr.next);
  }
}

}  // namespace sparrow
}  // namespace navigation
}  // namespace vtr
