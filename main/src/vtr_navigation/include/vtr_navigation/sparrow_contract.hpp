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
 * \file sparrow_contract.hpp
 * \brief Corridor contraction of the taught graph. Direct C++ port of
 *        vtr3_sim/pomcp/contract.py (+ the corner-vertex override from
 *        vtr3_sim/macro_nodes.py).
 *
 * WHY: POMCP's horizon is measured in ACTIONS, and a taught graph is mostly
 * degree-2 filler - sep12_4 is 664 vertices of which 645 are mid-corridor. A
 * search that spends a tree level on every 0.3 m micro-edge cannot see far
 * enough to compare routes. Contracting each degree-2 chain into one atomic
 * "drive this corridor" action collapses 664 vertices / 674 edges into 29
 * decision vertices / 39 corridors, which is a horizon the search can cover.
 *
 * The abstraction shortens the PLANNING horizon only. The executor still
 * reacts at taught-edge resolution: it drives the corridor checking micro-edges
 * in order and stops in front of a blockage that appeared mid-corridor, and
 * because `root` is always kept as an explicit macro-node, the replan from that
 * (non-junction) position is well defined.
 *
 * VERIFICATION GATE: on sep12_4 the taught graph of 664 vertices / 674
 * undirected edges must contract to exactly 29 decision vertices and 39
 * corridors. Every reported number depends on this; a graph that differs by one
 * decision vertex does not reproduce them. See test_sparrow_contract.cpp.
 */
#pragma once

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "vtr_navigation/sparrow_planner.hpp"

namespace vtr {
namespace navigation {
namespace sparrow {

/// One macro-edge: its endpoints, the micro chain, and its cost.
/// Ports contract.MacroEdge.
struct MacroEdge {
  SVertex u = 0;
  SVertex v = 0;
  /// The first micro vertex - i.e. what the executor actually drives towards.
  SVertex first_hop = 0;
  /// Full micro vertex sequence, EXCLUDING u and INCLUDING v (matches Python).
  std::vector<SVertex> chain;
  double travel_time = 0.0;

  size_t n_micro() const { return chain.size(); }
  SEdge edge() const { return canonical_edge(u, v); }
};

/// A contracted context plus the mapping back to micro actions.
/// Ports contract.MacroPlan.
struct MacroPlan {
  GraphContext context;
  std::map<SEdge, MacroEdge> macros;
  SVertex root = 0;

  /// First micro hop for driving the macro-edge (u, v); nullopt if unknown.
  std::optional<SVertex> toMicro(SVertex u, SVertex v) const;
  /// Micro-edge count of the macro-edge (u, v); 1 when not a macro-edge.
  size_t nMicro(SVertex u, SVertex v) const;
};

/**
 * \brief Contract `micro` into macro-edges.
 *
 * `root` (and any `extra_nodes`) are kept as explicit vertices so planning
 * starts exactly where the robot is - INCLUDING part-way down a corridor after
 * an interruption, which is what makes replan-on-interrupt well defined.
 *
 * \param max_macro_len_s  Cap on a single macro-edge's travel time; a longer
 *        corridor is split into ceil(L/cap) EQUAL pieces. 0 disables splitting.
 *        The deployment config uses 0: the corner-vertex rule already breaks up
 *        the long corridors on sep12_4.
 */
MacroPlan buildMacroPlan(const GraphContext& micro, SVertex root,
                         const std::set<SVertex>& extra_nodes = {},
                         double max_macro_len_s = 0.0);

/**
 * \brief Corner vertices named explicitly via POMCP_CORNER_VERTICES.
 *
 * Format: "run:vertex,run:vertex,..." e.g. "2:0,2:123,20:41". Each pair is
 * packed to an SVertex as (run << 32) | vertex, matching tactic::VertexId.
 *
 * Only the EXPLICIT override from macro_nodes.corner_nodes is ported. The
 * automatic rule there (degree-2 vertices whose +-span heading change exceeds
 * corner_deg, then the candidate nearest each bounding-box corner) needs 2D
 * vertex positions, which the planner's GraphContext does not carry - and the
 * handoff is explicit that it must NOT be used on sep12_4 anyway: the bbox
 * heuristic silently picks the wrong vertex on a non-rectangular building (the
 * real top-right bend at (14.13, 20.84) lost to a vertex 6.8 m from the bbox
 * corner against its own 7.8 m). Naming them is the only reliable option.
 *
 * Like the Python, a named vertex does NOT have to pass the turn test - it is
 * taken from the graph's vertices directly. Vertices named but absent from the
 * graph are reported through `missing` (the Python raises); the caller decides
 * whether that is fatal.
 */
std::set<SVertex> cornerVerticesFromEnv(const GraphContext& micro,
                                        std::vector<SVertex>* missing = nullptr);

/// Parse a "run:vertex,..." specification into packed vertex ids.
std::set<SVertex> parseCornerSpec(const std::string& spec);

/// Three-valued edge status, matching Navigator::computeAdjacentEdgeStatuses:
/// 1 = blocked, 0 = free (visibly clear), -1 = unknown (out of sensing range).
constexpr int kEdgeBlocked = 1;
constexpr int kEdgeFree = 0;
constexpr int kEdgeUnknown = -1;

/**
 * \brief Full micro-vertex path of a macro-edge, driven from `from`.
 *
 * Returns [from, ..., other_endpoint] inclusive, so consecutive pairs are
 * exactly the micro-edges the executor drives. `from` must be an endpoint;
 * an empty vector is returned otherwise.
 */
std::vector<SVertex> macroChainFrom(const MacroEdge& m, SVertex from);

/**
 * \brief The part of that chain still AHEAD of the robot.
 *
 * While driving a corridor, only blockages BETWEEN THE ROBOT AND THE TARGET
 * macro-node can stop it: a micro-edge the robot has already crossed is behind
 * it and must not be reported, or the robot would stop for an obstacle it has
 * passed and re-plan forever. `robot_at` is the last micro-vertex the robot has
 * reached; if it is not on the chain the whole chain is returned (conservative:
 * check everything).
 */
std::vector<SVertex> remainingChainFrom(const MacroEdge& m, SVertex from,
                                        SVertex robot_at);

/**
 * \brief Aggregate micro-edge statuses along a vertex path into one corridor
 *        status.
 *
 * A corridor is a series of micro-edges in the teach graph, so:
 *   - BLOCKED as soon as ANY micro-edge in it is blocked;
 *   - FREE only when EVERY micro-edge was actually observed free;
 *   - UNKNOWN otherwise.
 *
 * The three-valued rule is what makes this safe under partial observability.
 * The lidar only covers its sensing radius, so the far end of a long corridor
 * is genuinely unobserved and must stay UNKNOWN for the belief to price - the
 * handoff's "report only what lidar actually sees; the belief handles the
 * rest". Collapsing unknown into free is the failure the simulator measured as
 * catastrophic: the belief would drop a remembered mid-corridor blockage, the
 * search would price the corridor at nominal cost, and the robot would drive
 * into the same obstacle every episode.
 *
 * \param micro_status (u, v) -> one of kEdgeBlocked / kEdgeFree / kEdgeUnknown.
 */
int corridorStatus(const std::vector<SVertex>& chain,
                   const std::function<int(SVertex, SVertex)>& micro_status);

/**
 * \brief P(a remembered blockage is still there now), class-marginalised.
 *
 * A blocked sighting of observed age `a` seen `delta` seconds ago is still
 * blocked with probability S(a + delta) / S(a) under the frozen model - the
 * same conditioning the belief applies when it installs the memory into a
 * particle. Ports runner._memory_alive_prob.
 */
double memoryAliveProb(const EdgeMemoryRec& mem, double t_now,
                       const SparrowModel& model);

/// Micro-edges believed blocked right now, plus the vertices that make them
/// explicit in the abstract graph.
struct BlockedSet {
  std::set<SEdge> edges;
  std::set<SVertex> nodes;  ///< feed straight into buildMacroPlan's extra_nodes
};

/**
 * \brief The Marthi B-set. Ports runner._blocked_vertex_set.
 *
 * An edge belongs in the abstract graph when its probability of being blocked
 * exceeds `(1 + p_block) / 2` - i.e. it is much likelier blocked than a random
 * edge. Live sightings qualify at P=1; remembered sightings are
 * SURVIVAL-WEIGHTED, so a person seen 30 s ago drops out (its corridor keeps
 * the sighting at macro granularity) while a bin or tube seen minutes ago
 * stays an explicit planning vertex the search can wait at or route around.
 *
 * A remembered blockage on an edge the robot can currently SEE to be free is
 * discarded: the memory is stale.
 */
BlockedSet blockedVertexSet(const LocalObservation& local,
                            const SparrowModel& model);

/**
 * \brief Occupancy of a macro-edge spanning `n_micro` micro-edges.
 *
 * 1 - (1 - p)^n. This is a PROBABILITY, for the belief: a macro-edge is either
 * blocked or it is not. Rollout inflation is a different quantity - it uses
 * p * n, the expected NUMBER of blockages, because an expected cost must stay
 * linear in length. Conflating the two made a single-segment trap look safer
 * than an equally long many-segment ladder.
 */
double macroPBlock(double p_micro, size_t n_micro);

}  // namespace sparrow
}  // namespace navigation
}  // namespace vtr
