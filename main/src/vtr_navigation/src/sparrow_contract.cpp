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
 * \file sparrow_contract.cpp
 * \brief Port of vtr3_sim/pomcp/contract.py. See sparrow_contract.hpp.
 */
#include "vtr_navigation/sparrow_contract.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numeric>

namespace vtr {
namespace navigation {
namespace sparrow {

namespace {

/// First hop when driving the macro-edge from v back towards u.
/// chain is (u ->) ... -> v; driving the other way starts at the vertex
/// immediately before v, or u itself for a single-hop macro-edge.
/// Ports contract._reverse_first_hop.
SVertex reverseFirstHop(const MacroEdge& m) {
  return (m.chain.size() >= 2) ? m.chain[m.chain.size() - 2] : m.u;
}

/// v is a macro-node: explicitly kept, or a genuine decision vertex.
bool isNode(const GraphContext& micro, const std::set<SVertex>& keep,
            SVertex v) {
  return keep.count(v) != 0 || micro.is_decision_vertex(v);
}

const std::vector<SVertex>& neighborsOf(const GraphContext& micro, SVertex v) {
  static const std::vector<SVertex> kEmpty;
  auto it = micro.neighbors.find(v);
  return (it == micro.neighbors.end()) ? kEmpty : it->second;
}

/**
 * Extra macro-nodes so no macro-edge spans more than `max_len_s` seconds.
 * Ports contract._split_long_corridors.
 *
 * Each corridor is MEASURED FIRST and then divided into ceil(L/cap) EQUAL
 * pieces. Accumulating greedily and cutting whenever a running total crosses
 * the cap leaves the remainder as a stub - an 11.6 s corridor became 11.0 + 0.6,
 * i.e. a macro-node 0.3 m from a junction and a 1-micro-edge macro-edge, which
 * is both useless as a decision point and a wasted branch in the search.
 */
std::set<SVertex> splitLongCorridors(const GraphContext& micro,
                                     const std::set<SVertex>& keep,
                                     double max_len_s) {
  std::set<SVertex> extra;
  std::set<std::pair<SVertex, SVertex>> visited;

  std::vector<SVertex> starts;
  for (const auto& kv : micro.neighbors) {
    if (isNode(micro, keep, kv.first)) starts.push_back(kv.first);
  }

  for (const SVertex u : starts) {
    for (const SVertex first_hop : neighborsOf(micro, u)) {
      if (!visited.insert({u, first_hop}).second) continue;

      std::vector<SVertex> chain{u, first_hop};
      std::vector<double> cum{0.0, micro.edge_time(u, first_hop)};
      SVertex previous = u;
      SVertex current = first_hop;
      while (!isNode(micro, keep, current)) {
        std::vector<SVertex> options;
        for (const SVertex w : neighborsOf(micro, current)) {
          if (w != previous) options.push_back(w);
        }
        if (options.size() != 1) break;
        visited.insert({current, options[0]});
        previous = current;
        current = options[0];
        chain.push_back(current);
        cum.push_back(cum.back() + micro.edge_time(previous, current));
        if (current == u) break;
      }
      visited.insert({current, previous});

      const double total = cum.back();
      if (total <= max_len_s || chain.size() < 3) continue;

      const int pieces = static_cast<int>(std::ceil(total / max_len_s));
      std::set<size_t> used;
      for (int i = 1; i < pieces; ++i) {
        const double target = total * static_cast<double>(i) / pieces;
        // Interior indices ordered by |cum - target|, ties by index - exactly
        // the Python sort key (abs(cum[j] - target), j).
        std::vector<size_t> order(chain.size() >= 2 ? chain.size() - 2 : 0);
        std::iota(order.begin(), order.end(), size_t{1});
        std::stable_sort(order.begin(), order.end(),
                         [&](size_t a, size_t b) {
                           const double da = std::fabs(cum[a] - target);
                           const double db = std::fabs(cum[b] - target);
                           if (da != db) return da < db;
                           return a < b;
                         });
        for (const size_t j : order) {
          if (used.count(j) || isNode(micro, keep, chain[j])) continue;
          used.insert(j);
          extra.insert(chain[j]);
          break;
        }
      }
    }
  }
  return extra;
}

}  // namespace

std::optional<SVertex> MacroPlan::toMicro(SVertex u, SVertex v) const {
  auto it = macros.find(canonical_edge(u, v));
  if (it == macros.end()) return std::nullopt;
  const MacroEdge& m = it->second;
  return (m.u == u) ? m.first_hop : reverseFirstHop(m);
}

size_t MacroPlan::nMicro(SVertex u, SVertex v) const {
  auto it = macros.find(canonical_edge(u, v));
  return (it == macros.end()) ? size_t{1} : it->second.n_micro();
}

MacroPlan buildMacroPlan(const GraphContext& micro, SVertex root,
                         const std::set<SVertex>& extra_nodes,
                         double max_macro_len_s) {
  std::set<SVertex> keep{root, micro.goal};
  keep.insert(extra_nodes.begin(), extra_nodes.end());

  if (max_macro_len_s > 0.0) {
    const auto split = splitLongCorridors(micro, keep, max_macro_len_s);
    keep.insert(split.begin(), split.end());
  }

  std::map<SEdge, MacroEdge> macros;
  std::map<SVertex, std::vector<SVertex>> neighbors;
  std::map<SEdge, double> travel;

  // LIFO frontier and sorted neighbour iteration, matching the Python walk so
  // the orientation picked for each macro-edge is identical. (The macro SET is
  // order-independent - the min-cost tie-break below settles duplicates - but
  // matching the order keeps u/v/first_hop identical too, which makes the
  // cross-check against the simulator exact rather than merely equivalent.)
  std::vector<SVertex> frontier{root};
  std::set<SVertex> seen;
  while (!frontier.empty()) {
    const SVertex u = frontier.back();
    frontier.pop_back();
    if (!seen.insert(u).second) continue;
    neighbors.emplace(u, std::vector<SVertex>{});

    for (const SVertex first_hop : neighborsOf(micro, u)) {
      std::vector<SVertex> chain{first_hop};
      SVertex previous = u;
      SVertex current = first_hop;
      double cost = micro.edge_time(u, first_hop);
      while (!isNode(micro, keep, current)) {
        std::vector<SVertex> options;
        for (const SVertex w : neighborsOf(micro, current)) {
          if (w != previous) options.push_back(w);
        }
        if (options.size() != 1) break;
        previous = current;
        current = options[0];
        cost += micro.edge_time(previous, current);
        chain.push_back(current);
        if (current == u) break;
      }
      const SVertex v = current;
      if (v == u) continue;  // a loop back on itself is not a decision

      const SEdge key = canonical_edge(u, v);
      auto existing = macros.find(key);
      if (existing == macros.end() || cost < existing->second.travel_time) {
        MacroEdge m;
        m.u = u;
        m.v = v;
        m.first_hop = first_hop;
        m.chain = chain;
        m.travel_time = cost;
        macros[key] = std::move(m);
      }
      neighbors[u].push_back(v);
      neighbors[v].push_back(u);
      const double tt = macros[key].travel_time;
      travel[canonical_edge(u, v)] = tt;
      if (!seen.count(v)) frontier.push_back(v);
    }
  }

  MacroPlan plan;
  plan.context = GraphContext(std::move(neighbors), std::move(travel), micro.goal);
  plan.macros = std::move(macros);
  plan.root = root;
  return plan;
}

std::set<SVertex> parseCornerSpec(const std::string& spec) {
  std::set<SVertex> want;
  size_t pos = 0;
  while (pos <= spec.size()) {
    const size_t comma = spec.find(',', pos);
    std::string tok = spec.substr(
        pos, (comma == std::string::npos) ? std::string::npos : comma - pos);
    // trim
    const size_t b = tok.find_first_not_of(" \t");
    const size_t e = tok.find_last_not_of(" \t");
    if (b != std::string::npos) tok = tok.substr(b, e - b + 1);
    else tok.clear();

    if (!tok.empty()) {
      const size_t colon = tok.find(':');
      if (colon != std::string::npos) {
        try {
          const uint64_t run = std::stoull(tok.substr(0, colon));
          const uint64_t vid = std::stoull(tok.substr(colon + 1));
          // tactic::VertexId packing: (major << 32) | minor.
          want.insert((run << 32) | vid);
        } catch (const std::exception&) {
          // Python silently skips unparseable tokens (ValueError -> continue).
        }
      }
    }
    if (comma == std::string::npos) break;
    pos = comma + 1;
  }
  return want;
}

std::set<SVertex> cornerVerticesFromEnv(const GraphContext& micro,
                                        std::vector<SVertex>* missing) {
  const char* raw = std::getenv("POMCP_CORNER_VERTICES");
  if (raw == nullptr) return {};
  const std::set<SVertex> want = parseCornerSpec(raw);
  if (want.empty()) return {};

  std::set<SVertex> picked;
  for (const SVertex v : want) {
    // Taken from the graph's vertices directly: a NAMED corner does not have
    // to pass any turn test, matching macro_nodes.corner_nodes' override path
    // (which filters `nodes`, not the turn candidates `cand`).
    if (micro.neighbors.count(v)) picked.insert(v);
    else if (missing != nullptr) missing->push_back(v);
  }
  return picked;
}

std::vector<SVertex> macroChainFrom(const MacroEdge& m, SVertex from) {
  // m.chain is u -> ... -> v EXCLUDING u, so the full path is (u, chain...).
  std::vector<SVertex> full;
  full.reserve(m.chain.size() + 1);
  full.push_back(m.u);
  full.insert(full.end(), m.chain.begin(), m.chain.end());

  if (from == m.u) return full;
  if (from == m.v) {
    std::reverse(full.begin(), full.end());
    return full;
  }
  return {};  // not an endpoint
}

std::vector<SVertex> remainingChainFrom(const MacroEdge& m, SVertex from,
                                        SVertex robot_at) {
  std::vector<SVertex> full = macroChainFrom(m, from);
  if (full.empty()) return full;
  // Last occurrence: a corridor can in principle revisit a vertex, and the
  // robot is at the latest point it has reached, not the earliest.
  for (size_t i = full.size(); i-- > 0;) {
    if (full[i] == robot_at) {
      return std::vector<SVertex>(full.begin() + i, full.end());
    }
  }
  return full;  // robot not on this corridor: check all of it
}

int corridorStatus(const std::vector<SVertex>& chain,
                   const std::function<int(SVertex, SVertex)>& micro_status) {
  if (chain.size() < 2) return kEdgeUnknown;
  bool any_unknown = false;
  for (size_t i = 0; i + 1 < chain.size(); ++i) {
    const int s = micro_status(chain[i], chain[i + 1]);
    if (s == kEdgeBlocked) return kEdgeBlocked;  // one blockage blocks it
    if (s != kEdgeFree) any_unknown = true;
  }
  return any_unknown ? kEdgeUnknown : kEdgeFree;
}

double memoryAliveProb(const EdgeMemoryRec& mem, double t_now,
                       const SparrowModel& model) {
  const double delta = std::max(0.0, t_now - mem.t_obs);
  const double age0 = std::max(0.0, mem.age_at_obs);

  // A labelled sighting collapses the posterior onto its class; an unlabelled
  // one is marginalised over p(k | D > age0), exactly as the belief does.
  if (!mem.label.empty()) {
    return model.residual_survival(mem.label, delta, age0);
  }
  double acc = 0.0;
  for (const auto& kv : model.class_posterior_given_age(age0)) {
    if (kv.second > 0.0) {
      acc += kv.second * model.residual_survival(kv.first, delta, age0);
    }
  }
  return acc;
}

BlockedSet blockedVertexSet(const LocalObservation& local,
                            const SparrowModel& model) {
  BlockedSet out;
  const double p_micro = std::min(std::max(model.p_block, 0.0), 1.0);
  const double threshold = (1.0 + p_micro) / 2.0;

  // Live sightings: a blocked edge the robot can see is blocked at P = 1.
  for (const auto& kv : local.statuses) {
    if (kv.second != 0) out.edges.insert(kv.first);
  }

  for (const auto& kv : local.memory) {
    const SEdge& ce = kv.first;
    const EdgeMemoryRec& mem = kv.second;
    if (!mem.blocked || out.edges.count(ce)) continue;
    // Currently visible and free: the memory is stale, drop it.
    auto sit = local.statuses.find(ce);
    if (sit != local.statuses.end() && sit->second == 0) continue;
    if (memoryAliveProb(mem, local.time, model) >= threshold) {
      out.edges.insert(ce);
    }
  }

  for (const auto& e : out.edges) {
    out.nodes.insert(e.first);
    out.nodes.insert(e.second);
  }
  return out;
}

double macroPBlock(double p_micro, size_t n_micro) {
  const double p = std::min(std::max(p_micro, 0.0), 1.0);
  const size_t n = std::max<size_t>(n_micro, 1);
  return 1.0 - std::pow(1.0 - p, static_cast<double>(n));
}

}  // namespace sparrow
}  // namespace navigation
}  // namespace vtr
