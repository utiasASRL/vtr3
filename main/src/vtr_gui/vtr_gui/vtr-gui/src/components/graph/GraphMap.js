/**
 * Copyright 2021, Autonomous Space Robotics Lab (ASRL)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

import React from "react";

import "leaflet/dist/leaflet.css";
import L from "leaflet";
import "leaflet-rotatedmarker"; // enable marker rotation
import { MapContainer, TileLayer, ZoomControl } from "react-leaflet";
import { kdTree } from "kd-tree-javascript";

import ToolsMenu from "../tools/ToolsMenu";
import GoalManager from "../goal/GoalManager";

import NewGoalWaypointSVG from "../../images/new-goal-waypoint.svg";
import RunningGoalWaypointSVG from "../../images/running-goal-waypoint.svg";
import RobotIconSVG from "../../images/arrow.svg";
import TargetIconSVG from "../../images/arrow-merge.svg";
import SelectorCenterSVG from "../../images/selector-center.svg";
import SelectorEndSVG from "../../images/selector-end.svg";
import SelectorStartSVG from "../../images/selector-start.svg";
import MoveGraphTranslationSvg from "../../images/move-graph-translation.svg";
import MoveGraphRotationSvg from "../../images/move-graph-rotation.svg";
import MoveGraphScaleSvg from "../../images/move-graph-scale.svg";

/// pose graph constants
const ROUTE_TYPE_COLOR = ["#f44336", "#ff9800", "#ffeb3b", "#4caf50", "#00bcd4", "#2196f3", "#9c27b0"];
const GRAPH_OPACITY = 0.9;
const GRAPH_WEIGHT = 7;

/// robot constants
const ROBOT_OPACITY = 0.8;
const ROBOT_UNLOCALIZED_OPACITY = 0.2;
const ROBOT_ICON = new L.Icon({
  iconUrl: RobotIconSVG,
  iconSize: new L.Point(30, 30),
});
const TARGET_ICON = new L.Icon({
  iconUrl: TargetIconSVG,
  iconSize: new L.Point(30, 30),
});

/// following route constants
const FOLLOWING_ROUTE_OPACITY = 1.0;
const FOLLOWING_ROUTE_WEIGHT = 4;

///
const NEW_GOAL_WAYPOINT_ICON = new L.Icon({
  iconUrl: NewGoalWaypointSVG,
  iconAnchor: [15, 30],
  iconSize: new L.Point(30, 30),
});
const RUNNING_GOAL_WAYPOINT_ICON = new L.Icon({
  iconUrl: RunningGoalWaypointSVG,
  iconAnchor: [15, 30],
  iconSize: new L.Point(30, 30),
});
const WAYPOINT_OPACITY = 0.9;

/// selector constants
const SELECTOR_CENTER_ICON = new L.Icon({
  iconUrl: SelectorCenterSVG,
  iconSize: new L.Point(30, 30),
});
const SELECTOR_END_ICON = new L.Icon({
  iconUrl: SelectorEndSVG,
  iconSize: new L.Point(40, 40),
});
const SELECTOR_START_ICON = new L.Icon({
  iconUrl: SelectorStartSVG,
  iconSize: new L.Point(40, 40),
});

/// move graph constants
const MOVE_GRAPH_TRANSLATION_ICON = new L.Icon({
  iconUrl: MoveGraphTranslationSvg,
  iconSize: new L.Point(40, 40),
});
const MOVE_GRAPH_ROTATION_ICON = new L.Icon({
  iconUrl: MoveGraphRotationSvg,
  iconSize: new L.Point(40, 40),
});
const MOVE_GRAPH_SCALE_ICON = new L.Icon({
  iconUrl: MoveGraphScaleSvg,
  iconSize: new L.Point(40, 40),
});
const MOVE_GRAPH_MARKER_OPACITY = 0.9; // translation and rotation
const MOVE_GRAPH_MARKER_OPACITY2 = 0.2; // scale

/// move robot constants
const MOVE_ROBOT_OPACITY = 0.5;

class GraphMap extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      /// goal manager
      server_state: "EMPTY",
      goals: [], // {id, type <teach,repeat>, waypoints, pause_before, pause_after}
      curr_goal_idx: -1,
      new_goal_type: "",
      new_goal_waypoints: [],
      following_route_ids: [],
      /// tools menu
      current_tool: null,
      // annotate route
      annotate_route_type: 0,
      annotate_route_ids: [],
      // merge
      merge_ids: [],
      // move graph
      move_graph_change: { lng: 0, lat: 0, theta: 0, scale: 1 },
      // move robot
      move_robot_vertex: { lng: 0, lat: 0, id: -1 },
    };

    /// leaflet map
    this.map = null; // leaflet map instance

    /// pose graph states
    this.graph_loaded = false; // flag when graph has been loaded and ready to accept updates
    this.root_vid = null; // root vertex id  /// \todo get from server
    this.id2vertex = null; // id2vertex map
    this.kdtree = null; // kdtree for fast nearest neighbor search (created on demand)
    this.fixed_routes = [];
    this.active_routes = [];

    /// robot state
    this.robot_marker = {
      valid: false,
      marker: L.marker([0, 0], {
        draggable: false,
        icon: ROBOT_ICON,
        opacity: ROBOT_OPACITY,
        pane: "graph",
        zIndexOffset: 100,
        rotationOrigin: "center",
        rotationAngle: 0,
      }),
    };
    this.target_marker = {
      valid: false,
      marker: L.marker([0, 0], {
        draggable: false,
        icon: TARGET_ICON,
        opacity: ROBOT_OPACITY,
        pane: "graph",
        zIndexOffset: 200,
        rotationOrigin: "center",
        rotationAngle: 0,
      }),
    };
    this.target_marker.marker.on("click", () => this.props.socket.emit("command/confirm_merge", {}));

    /// following route
    this.following_route_polyline = null;

    /// goal manager
    this.new_goal_markers = [];
    this.running_goal_markers = [];
  }

  componentDidMount() {
    // Socket IO
    this.props.socket.on("graph/state", this.graphStateCallback.bind(this));
    this.props.socket.on("graph/update", this.graphUpdateCallback.bind(this));
    this.props.socket.on("robot/state", this.robotStateCallback.bind(this));
    this.props.socket.on("following_route", this.followingRouteCallback.bind(this));
    this.props.socket.on("mission/server_state", this.serverStateCallback.bind(this));
  }

  componentWillUnmount() {
    // Socket IO
    this.props.socket.off("graph/state", this.graphStateCallback.bind(this));
    this.props.socket.off("graph/update", this.graphUpdateCallback.bind(this));
    this.props.socket.off("robot/state", this.robotStateCallback.bind(this));
    this.props.socket.off("following_route", this.followingRouteCallback.bind(this));
    this.props.socket.off("mission/server_state", this.serverStateCallback.bind(this));
  }

  render() {
    const { socket } = this.props;
    const {
      server_state,
      goals,
      curr_goal_idx,
      new_goal_type,
      new_goal_waypoints,
      following_route_ids,
      current_tool,
      annotate_route_type,
      annotate_route_ids,
      merge_ids,
      move_graph_change,
      move_robot_vertex,
    } = this.state;

    return (
      <>
        {/* Leaflet map container with initial center set to UTIAS (only for initialization) */}
        <MapContainer
          center={[43.782, -79.466]}
          zoom={18}
          zoomControl={false}
          whenCreated={this.mapCreatedCallback.bind(this)}
        >
          {/* Leaflet map background */}
          <TileLayer
            // url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" // load from OSM directly
            url="/tile/{s}/{x}/{y}/{z}" // load from backend (potentially cached)
            maxZoom={20}
          />
          <ZoomControl position="bottomright" />
        </MapContainer>
        <ToolsMenu
          socket={socket}
          currentTool={current_tool}
          selectTool={this.selectTool.bind(this)}
          deselectTool={this.deselectTool.bind(this)}
          // annotate route
          onSliderChange={this.handleAnnotateRouteSliderChange.bind(this)}
          annotateRouteType={annotate_route_type}
          annotateRouteIds={annotate_route_ids}
          // move graph
          moveGraphChange={move_graph_change}
          // move robot
          moveRobotVertex={move_robot_vertex}
        />
        <GoalManager
          socket={socket}
          currentTool={current_tool}
          selectTool={this.selectTool.bind(this)}
          deselectTool={this.deselectTool.bind(this)}
          serverState={server_state}
          goals={goals}
          currGoalIdx={curr_goal_idx}
          newGoalType={new_goal_type}
          setNewGoalType={this.setNewGoalType.bind(this)}
          newGoalWaypoints={new_goal_waypoints}
          setNewGoalWaypoints={this.setNewGoalWaypoints.bind(this)}
          setRunningGoalWaypoints={this.setRunningGoalWaypoints.bind(this)}
          followingRouteIds={following_route_ids}
          // merge
          mergeIds={merge_ids}
        />
      </>
    );
  }

  /** @brief Leaflet map creationg callback */
  mapCreatedCallback(map) {
    console.debug("Leaflet map created.");
    //
    this.map = map;
    this.map.createPane("graph"); // used for the graph polylines and robot marker
    map.getPane("graph").style.zIndex = 500; // same as the shadow pane (polylines default)
    //
    this.map.on("click", this.handleMapClick.bind(this));
    //
    this.fetchGraphState(true);
  }

  /**
   * @brief Returns the closes vertex on graph according to user selected latlng with some tolerance.
   * Helper function of handleMapClick.
   * @param {Object} latlng User selected lat and lng.
   * @param {number} tol Tolerance in terms of the window.
   */
  getClosestPoint(latlng, tol = 0.02) {
    //
    let bounds = this.map.getBounds();
    let max_dist = Math.max(bounds.getSouthWest().distanceTo(bounds.getNorthEast()) * tol, 1);
    //
    if (this.kdtree === null) return { target: null, distance: max_dist };
    //
    let res = this.kdtree.nearest(latlng, 1, max_dist);
    if (res.length > 0) return { target: res[0][0], distance: res[0][1] };
    else return { target: null, distance: max_dist };
  }

  handleMapClick(e) {
    // Find the closest vertex
    let best = this.getClosestPoint(e.latlng);
    if (best.target === null) return;
    this.setState((state) => {
      if (state.new_goal_type === "repeat") {
        let waypoint_marker = L.marker(this.id2vertex.get(best.target.id), {
          draggable: false,
          icon: NEW_GOAL_WAYPOINT_ICON,
          opacity: WAYPOINT_OPACITY,
          pane: "graph",
        });
        waypoint_marker.addTo(this.map);
        this.new_goal_markers.push(waypoint_marker);
        return { new_goal_waypoints: [...state.new_goal_waypoints, best.target.id] };
      }
    });
  }

  fetchGraphState(center = false) {
    console.info("Fetching the current pose graph state (full).");
    fetch("/vtr/graph")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch pose graph state: " + response.status);
        response.json().then((data) => {
          console.info("Received the pose graph state (full): ", data);
          this.loadGraphState(data, center);
          // fetch robot and following route after graph has been set
          this.fetchRobotState();
          this.fetchFollowingRoute();
          this.fetchServerState();
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  graphStateCallback(graph_state) {
    console.info("Received graph state: ", graph_state);
    this.loadGraphState(graph_state);
  }

  /** @brief Helper function to convert a pose graph route to a leaflet polyline, and add it to map */
  route2Polyline(route) {
    // fixed_routes format: [{type: 0, ids: [id, ...]}, ...]
    let color = ROUTE_TYPE_COLOR[route.type % ROUTE_TYPE_COLOR.length];
    let latlngs = route.ids.map((id) => {
      let v = this.id2vertex.get(id);
      return [v.lat, v.lng];
    });
    let polyline = L.polyline(latlngs, { color: color, opacity: GRAPH_OPACITY, weight: GRAPH_WEIGHT, pane: "graph" });
    polyline.addTo(this.map);
    return polyline;
  }

  /** @brief Refresh the pose graph completely */
  loadGraphState(graph, center = false) {
    console.info("Loading the current pose graph state (full).");
    // root vid
    this.root_vid = 0; // = graph.root_vid; /// \todo
    // id2vertex and kdtree
    this.id2vertex = new Map();
    graph.vertices.forEach((v) => {
      v.valueOf = () => v.id;
      v.distanceTo = L.LatLng.prototype.distanceTo;
      this.id2vertex.set(v.id, v);
    });
    this.kdtree = new kdTree(graph.vertices, (a, b) => b.distanceTo(a), ["lat", "lng"]);
    // fixed routes
    this.fixed_routes.forEach((route) => {
      route.polyline.remove();
    });
    this.fixed_routes = graph.fixed_routes.map((route) => {
      let polyline = this.route2Polyline(route);
      return { ...route, polyline: polyline };
    });
    // active routes
    this.active_routes.forEach((route) => {
      route.polyline.remove();
    });
    this.active_routes = graph.active_routes.map((route) => {
      let polyline = this.route2Polyline(route);
      return { ...route, polyline: polyline };
    });

    // center set to root vertex
    if (center && this.id2vertex.has(this.root_vid)) {
      let v = this.id2vertex.get(this.root_vid);
      this.map.flyTo([v.lat, v.lng]);
    }

    this.graph_loaded = true;
  }

  fetchRobotState() {
    console.info("Fetching the current robot state (full).");
    fetch("/vtr/robot")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch robot state: " + response.status);
        response.json().then((data) => {
          console.info("Received the robot state (full): ", data);
          this.loadRobotState(data);
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  robotStateCallback(robot_state) {
    console.info("Received robot state: ", robot_state);
    this.loadRobotState(robot_state);
  }

  loadRobotState(robot) {
    if (this.map === null) return;
    if (this.graph_loaded === false) return;
    console.info("Loading the current robot state: ", robot);
    //
    if (robot.valid === false) {
      if (this.robot_marker.valid === true) {
        this.robot_marker.valid = false;
        this.robot_marker.marker.remove();
      }
    } else {
      this.robot_marker.marker.setLatLng({ lng: robot.lng, lat: robot.lat });
      this.robot_marker.marker.setOpacity(robot.localized ? ROBOT_OPACITY : ROBOT_UNLOCALIZED_OPACITY);
      this.robot_marker.marker.setRotationAngle(-(robot.theta / Math.PI) * 180);
      if (this.robot_marker.valid === false) {
        this.robot_marker.valid = true;
        this.robot_marker.marker.addTo(this.map);
      }
    }
    //
    if (robot.target_valid === false) {
      if (this.target_marker.valid === true) {
        this.target_marker.valid = false;
        this.target_marker.marker.remove();
      }
    } else {
      this.target_marker.marker.setLatLng({ lng: robot.target_lng, lat: robot.target_lat });
      this.target_marker.marker.setOpacity(robot.target_localized ? ROBOT_OPACITY : ROBOT_UNLOCALIZED_OPACITY);
      this.target_marker.marker.setRotationAngle(-(robot.target_theta / Math.PI) * 180);
      if (this.target_marker.valid === false) {
        this.target_marker.valid = true;
        this.target_marker.marker.addTo(this.map);
      }
    }
  }

  fetchFollowingRoute() {
    console.info("Fetching the current following route.");
    fetch("/vtr/following_route")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch following route: " + response.status);
        response.json().then((data) => {
          console.info("Received the following route: ", data);
          this.loadFollowingRoute(data);
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  followingRouteCallback(following_route) {
    console.info("Received following route: ", following_route);
    this.loadFollowingRoute(following_route);
  }

  loadFollowingRoute(following_route) {
    if (this.map === null) return;
    if (this.graph_loaded === false) return;
    console.info("Loading the current following route: ", following_route);
    //
    if (this.following_route_polyline !== null) this.following_route_polyline.remove();
    //
    if (following_route.ids.length === 0) {
      this.following_route_polyline = null;
      this.setState({ following_route_ids: [] });
    } else {
      let latlngs = following_route.ids.map((id) => {
        let v = this.id2vertex.get(id);
        return [v.lat, v.lng];
      });
      let polyline = L.polyline(latlngs, {
        color: "#FFFFFF",
        opacity: FOLLOWING_ROUTE_OPACITY,
        weight: FOLLOWING_ROUTE_WEIGHT,
        pane: "graph",
      });
      polyline.addTo(this.map);
      this.following_route_polyline = polyline;
      this.setState({ following_route_ids: following_route.ids });
    }
  }

  fetchServerState() {
    console.info("Fetching the current server state (full).");
    fetch("/vtr/server")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch server state: " + response.status);
        response.json().then((data) => {
          console.info("Received the server state (full): ", data);
          this.loadServerState(data);
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  serverStateCallback(data) {
    console.info("Received the server state (full): ", data);
    this.loadServerState(data);
  }

  loadServerState(data) {
    console.info("Loading the current server state: ", data);
    let curr_goal_idx = -1;
    for (let i = 0; i < data.goals.length; i++) {
      if (data.goals[i].id.toString() === data.current_goal_id.toString()) {
        curr_goal_idx = i;
        break;
      }
    }
    this.setRunningGoalWaypoints(curr_goal_idx === -1 ? [] : data.goals[curr_goal_idx].waypoints);
    this.setState({ server_state: data.server_state, goals: data.goals, curr_goal_idx: curr_goal_idx });
  }

  graphUpdateCallback(graph_update) {
    if (this.map === null) return;
    if (this.graph_loaded === false) return;
    console.info("Received graph update: ", graph_update);

    // from vertex
    let vf = graph_update.vertex_from;
    vf.valueOf = () => vf.id;
    vf.distanceTo = L.LatLng.prototype.distanceTo;
    // only update if the vertex is not in the map (vertex position does not change)
    if (!this.id2vertex.has(vf.id)) this.kdtree.insert(vf);
    // always update the vertex map, because vertex neighbors may change
    this.id2vertex.set(vf.id, vf);

    // to vertex
    let vt = graph_update.vertex_to;
    vt.valueOf = () => vt.id;
    vt.distanceTo = L.LatLng.prototype.distanceTo;
    // only update if the vertex is not in the map (vertex position does not change)
    if (!this.id2vertex.has(vt.id)) this.kdtree.insert(vt);
    // always update the vertex map, because vertex neighbors may change
    this.id2vertex.set(vt.id, vt);

    // active route update
    if (this.active_routes.length === 0) {
      let active_route = { ids: [vf.id], type: vf.type };
      active_route = { ...active_route, polyline: this.route2Polyline(active_route) };
      this.active_routes.push(active_route);
    }
    let active_route = this.active_routes[this.active_routes.length - 1];
    active_route.ids.push(vt.id);
    active_route.polyline.addLatLng([vt.lat, vt.lng]);
    if (active_route.type !== vt.type) {
      let new_active_route = { ids: [vt.id], type: vt.type };
      new_active_route = { ...new_active_route, polyline: this.route2Polyline(new_active_route) };
      this.active_routes.push(new_active_route);
    }
  }

  /**
   * @brief Sets the type of the current goal being added.
   * @param {string} type Type of the goal being added <teach, repeat>
   */
  setNewGoalType(type) {
    console.info("Setting new goal type: ", type);
    this.setState({ new_goal_type: type });
  }

  /**
   * @brief Sets the target vertices of the current goal being added. For repeat only.
   * @param {array} waypoints Array of vertex ids indicating the repeat path.
   */
  setNewGoalWaypoints(ids) {
    console.info("Setting new goal waypoints: ", ids);
    this.new_goal_markers.forEach((marker) => marker.remove());
    this.new_goal_markers = ids.map((id) => {
      let waypoint_marker = L.marker(this.id2vertex.get(id), {
        draggable: false,
        icon: NEW_GOAL_WAYPOINT_ICON,
        opacity: WAYPOINT_OPACITY,
        pane: "graph",
      });
      waypoint_marker.addTo(this.map);
      return waypoint_marker;
    });
    this.setState({ new_goal_waypoints: ids });
  }

  /**
   * @brief Sets the target vertices of the running goal. For repeat only.
   * @param {array} waypoints Array of vertex ids indicating the repeat path.
   */
  setRunningGoalWaypoints(ids) {
    console.info("Setting running goal waypoints: ", ids);
    this.running_goal_markers.forEach((marker) => marker.remove());
    this.running_goal_markers = ids.map((id) => {
      let waypoint_marker = L.marker(this.id2vertex.get(id), {
        draggable: false,
        icon: RUNNING_GOAL_WAYPOINT_ICON,
        opacity: WAYPOINT_OPACITY,
        pane: "graph",
      });
      waypoint_marker.addTo(this.map);
      return waypoint_marker;
    });
  }

  /**
   * @brief Selects the corresponding tool based on user inputs.
   * @param {string} tool The tool that user selects.
   */
  selectTool(tool) {
    this.setState((state) => {
      if (tool === state.current_tool) return;
      console.info("Selecting tool: ", tool);
      // deselect the current tool first
      if (state.current_tool !== null) {
        switch (tool) {
          case "annotate_route":
            this.finishAnnotateRoute();
            break;
          case "move_graph":
            this.finishMoveGraph();
            break;
          case "move_robot":
            this.finishMoveRobot();
            break;
          case "merge":
            this.finishMerge();
            break;
          default:
            break;
        }
      }
      // select the new tool
      switch (tool) {
        case "annotate_route":
          this.startAnnotateRoute();
          break;
        case "move_graph":
          this.startMoveGraph();
          break;
        case "move_robot":
          this.startMoveRobot();
          break;
        case "merge":
          this.startMerge();
          break;
        default:
          break;
      }
      //
      return { current_tool: tool };
    });
  }

  /** @brief De-selects the current tool. */
  deselectTool() {
    this.setState((state) => {
      if (state.current_tool === null) return;
      console.info("deselecting tool: ", state.current_tool);
      // deselect the current tool first
      switch (state.current_tool) {
        case "annotate_route":
          this.finishAnnotateRoute();
          break;
        case "move_graph":
          this.finishMoveGraph();
          break;
        case "move_robot":
          this.finishMoveRobot();
          break;
        case "merge":
          this.finishMerge();
          break;
        default:
          break;
      }
      //
      return { current_tool: null };
    });
  }

  /**
   * @brief Helper function to find user selected paths for merging.
   * @param {int} source Starting vertex id.
   * @param {array} dests Array of destination vertex ids (start & end).
   */
  breadthFirstSearch(source, dests) {
    let queue = [source];
    let done = new Set();

    let parents = new Map();
    parents.set(source, -1);

    let targets = new Set(dests);

    while (queue.length > 0 && targets.size > 0) {
      let node = queue.shift();
      if (done.has(node)) continue;
      done.add(node);
      this.id2vertex.get(node).neighbors.forEach((neighbor) => {
        if (!parents.has(neighbor)) parents.set(neighbor, node);
        targets.delete(neighbor);
        queue.push(neighbor);
      });
    }

    let paths = [];
    dests.forEach((target) => {
      let path = [];
      let current = target;

      // Test to make sure we found the target
      if (!targets.has(target)) {
        while (current !== -1) {
          path.unshift(current);
          current = parents.get(current);
        }
      }
      paths.push(path);
    });

    return paths;
  }

  /** @brief */
  createSelector(selector, callback) {
    // Initial position of the start, center and end markers based on current map.
    // Offset to start/end marker is ~10% of the screen diagonal in each direction.
    let map_bounds = this.map.getBounds();
    let sw = map_bounds.getSouthWest();
    let ne = map_bounds.getNorthEast();
    let offs = (Math.abs(sw.lat - ne.lat) + Math.abs(sw.lng - ne.lng)) / 20;
    let center_pos = this.map.getCenter();
    let start_pos = {
      lat: center_pos.lat,
      lng: center_pos.lng - offs,
    };
    let end_pos = {
      lat: center_pos.lat,
      lng: center_pos.lng + offs,
    };

    // Intermediate position of each maker that gets updated while dragging.
    let interm_pos = { s: null, c: null, e: null };

    let getSelectedPath = () => {
      let paths = this.breadthFirstSearch(selector.vertex.c.id, [selector.vertex.s.id, selector.vertex.e.id]);
      paths[0].reverse();
      return paths[0].concat(paths[1].slice(1));
    };

    let getRotationAngle = (p1, p2) => {
      let point1 = this.map.project(this.id2vertex.get(p1));
      let point2 = this.map.project(this.id2vertex.get(p2));
      let diff = point2.subtract(point1);
      return 90 - (Math.atan2(diff.x, diff.y) * 180) / Math.PI;
    };

    // Drag handles for start and end markers.
    let handleDrag = (e, key) => (interm_pos[key] = e.latlng);
    let handleDragEnd = (key) => {
      let closest_vertices = this.kdtree.nearest(interm_pos[key], 1);
      selector.vertex[key] = closest_vertices ? closest_vertices[0][0] : selector.vertex[key];

      // Find paths
      let selected_path = getSelectedPath();

      // Set new marker location
      let rotation_angle =
        selected_path.length < 2
          ? 0 // No enough vertices to calculate angle
          : key === "s"
          ? getRotationAngle(selected_path[1], selected_path[0])
          : getRotationAngle(selected_path[selected_path.length - 1], selected_path[selected_path.length - 2]);
      selector.marker[key].setLatLng(selector.vertex[key]);
      selector.marker[key].setRotationAngle(rotation_angle);
      interm_pos[key] = null;

      console.info("Selected path is: ", selected_path);
      callback(selected_path);
    };

    // Drag handles for the center marker. The start and end markers move with
    // it while it is dragging.
    let handleDragC = (e) => {
      interm_pos.c = e.latlng;
      interm_pos.s = {
        lat: selector.vertex.s.lat + e.latlng.lat - selector.vertex.c.lat,
        lng: selector.vertex.s.lng + e.latlng.lng - selector.vertex.c.lng,
      };
      selector.marker.s.setLatLng(interm_pos.s);
      interm_pos.e = {
        lat: selector.vertex.e.lat + e.latlng.lat - selector.vertex.c.lat,
        lng: selector.vertex.e.lng + e.latlng.lng - selector.vertex.c.lng,
      };
      selector.marker.e.setLatLng(interm_pos.e);
    };
    let handleDragEndC = () => {
      let closest_vertices = this.kdtree.nearest(interm_pos.c, 1);
      selector.vertex.c = closest_vertices ? closest_vertices[0][0] : selector.vertex.c;
      selector.marker.c.setLatLng(selector.vertex.c);
      interm_pos.c = null;
      handleDragEnd("s");
      handleDragEnd("e");
    };

    // set up and add the center marker
    let closest_vertices = this.kdtree.nearest(center_pos, 1);
    selector.vertex.c = closest_vertices ? closest_vertices[0][0] : center_pos;
    selector.marker.c = L.marker(selector.vertex.c, {
      draggable: true,
      icon: SELECTOR_CENTER_ICON,
      opacity: GRAPH_OPACITY,
      zIndexOffset: 100,
      pane: "graph",
    });
    selector.marker.c.on("drag", (e) => handleDragC(e));
    selector.marker.c.on("dragend", () => handleDragEndC());
    selector.marker.c.addTo(this.map);

    // Compute merge path here first for calculating rotation angle of start and end marker.
    closest_vertices = this.kdtree.nearest(start_pos, 1);
    selector.vertex.s = closest_vertices ? closest_vertices[0][0] : start_pos;
    closest_vertices = this.kdtree.nearest(end_pos, 1);
    selector.vertex.e = closest_vertices ? closest_vertices[0][0] : end_pos;
    let selected_path = getSelectedPath();
    // The start marker
    selector.marker.s = L.marker(selector.vertex.s, {
      draggable: true,
      icon: SELECTOR_START_ICON,
      opacity: GRAPH_OPACITY,
      zIndexOffset: 200,
      pane: "graph",
      rotationOrigin: "center",
      rotationAngle: selected_path.length > 1 ? getRotationAngle(selected_path[1], selected_path[0]) : 0,
    });
    selector.marker.s.on("drag", (e) => handleDrag(e, "s"));
    selector.marker.s.on("dragend", () => handleDragEnd("s"));
    selector.marker.s.addTo(this.map);
    // The end marker.
    selector.marker.e = L.marker(selector.vertex.e, {
      draggable: true,
      icon: SELECTOR_END_ICON,
      opacity: GRAPH_OPACITY,
      zIndexOffset: 200,
      pane: "graph",
      rotationOrigin: "center",
      rotationAngle:
        selected_path.length > 1
          ? getRotationAngle(selected_path[selected_path.length - 1], selected_path[selected_path.length - 2])
          : 0,
    });
    selector.marker.e.on("drag", (e) => handleDrag(e, "e"));
    selector.marker.e.on("dragend", () => handleDragEnd("e"));
    selector.marker.e.addTo(this.map);

    console.info("Selected path is: ", selected_path);
    callback(selected_path);
  }

  removeSelector(selector) {
    selector.marker.s.remove();
    selector.marker.c.remove();
    selector.marker.e.remove();
  }

  /// Merge
  startMerge() {
    console.info("Start merging");
    this.merge_selector = { marker: { s: null, c: null, e: null }, vertex: { s: null, c: null, e: null } };
    this.merge_polyline = null;
    let selectPathCallback = (ids) => {
      this.setState({ merge_ids: ids }, () => {
        let latlngs = ids.map((id) => {
          let v = this.id2vertex.get(id);
          return [v.lat, v.lng];
        });
        if (this.merge_polyline === null) {
          this.merge_polyline = L.polyline(latlngs, {
            color: "#FFFFFF",
            opacity: FOLLOWING_ROUTE_OPACITY,
            weight: FOLLOWING_ROUTE_WEIGHT,
            pane: "graph",
          });
          this.merge_polyline.addTo(this.map);
        } else {
          this.merge_polyline.setLatLngs(latlngs);
        }
      });
    };
    this.createSelector(this.merge_selector, selectPathCallback);
  }

  finishMerge() {
    console.info("Finish merging");
    if (this.merge_polyline !== null) {
      this.merge_polyline.remove();
      this.merge_polyline = null;
    }
    if (this.merge_selector !== null) {
      this.removeSelector(this.merge_selector);
      this.merge_selector = null;
    }
  }

  /// Annotate Route
  startAnnotateRoute() {
    console.info("Start annotating route");
    this.annotate_route_selector = { marker: { s: null, c: null, e: null }, vertex: { s: null, c: null, e: null } };
    this.annotate_polyline = null;
    let selectPathCallback = (ids) => {
      this.setState({ annotate_route_ids: ids }, () => {
        let latlngs = ids.map((id) => {
          let v = this.id2vertex.get(id);
          return [v.lat, v.lng];
        });
        if (this.annotate_polyline === null) {
          this.annotate_polyline = L.polyline(latlngs, {
            color: ROUTE_TYPE_COLOR[0],
            opacity: GRAPH_OPACITY,
            weight: GRAPH_WEIGHT,
            pane: "graph",
          });
          this.annotate_polyline.addTo(this.map);
        } else {
          this.annotate_polyline.setLatLngs(latlngs);
        }
      });
    };
    this.createSelector(this.annotate_route_selector, selectPathCallback);
  }

  finishAnnotateRoute() {
    console.info("Finish annotating route");
    if (this.annotate_polyline !== null) {
      this.annotate_polyline.remove();
      this.annotate_polyline = null;
    }
    if (this.annotate_route_selector !== null) {
      this.removeSelector(this.annotate_route_selector);
      this.annotate_route_selector = null;
    }
  }

  handleAnnotateRouteSliderChange(type) {
    if (this.annotate_polyline === null) return;
    this.setState({ annotate_route_type: type }, () =>
      this.annotate_polyline.setStyle({ color: ROUTE_TYPE_COLOR[type % ROUTE_TYPE_COLOR.length] })
    );
  }

  /// Move Graph
  startMoveGraph() {
    console.info("Start moving graph");
    let vid = this.root_vid;
    let origin = L.latLng(this.id2vertex.get(vid));
    let trans_loc = L.latLng(this.id2vertex.get(vid));
    // Marker for translating the graph
    this.trans_marker = L.marker(trans_loc, {
      draggable: true,
      zIndexOffset: 1000,
      icon: MOVE_GRAPH_TRANSLATION_ICON,
      opacity: MOVE_GRAPH_MARKER_OPACITY,
    });

    let p_center = this.map.latLngToLayerPoint(trans_loc);
    let p_bounds = this.map.getPixelBounds();
    let unit_scale_p = (p_bounds.max.x - p_bounds.min.x + p_bounds.max.y - p_bounds.min.y) / 16.0;

    // Marker that indicates scale change (basically a circle at the same
    // location as the translation marker), for visualization only
    this.scale_marker = L.marker(trans_loc, {
      draggable: false,
      icon: MOVE_GRAPH_SCALE_ICON,
      opacity: MOVE_GRAPH_MARKER_OPACITY2,
    });
    // adjust the size of the marker so that it connects the trans and rot
    // marker
    let icon = this.scale_marker.options.icon;
    icon.options.iconSize = [2 * unit_scale_p, 2 * unit_scale_p];
    this.scale_marker.setIcon(icon);

    let rot_loc = this.map.layerPointToLatLng(p_center.add(L.point(0, unit_scale_p)));
    // Marker for rotating the graph
    this.rot_marker = L.marker(rot_loc, {
      draggable: true,
      zIndexOffset: 1100,
      icon: MOVE_GRAPH_ROTATION_ICON,
      opacity: MOVE_GRAPH_MARKER_OPACITY,
    });

    let computeMoveGraphUpdate = () => {
      let trans_loc_p = this.map.latLngToLayerPoint(trans_loc);
      let rot_loc_p = this.map.latLngToLayerPoint(rot_loc);
      let diff_p = rot_loc_p.subtract(trans_loc_p);
      let theta = Math.atan2(diff_p.x, diff_p.y);
      let scale = Math.sqrt(Math.pow(diff_p.x, 2) + Math.pow(diff_p.y, 2)) / unit_scale_p;
      let move_graph_change = {
        lng: trans_loc.lng - origin.lng,
        lat: trans_loc.lat - origin.lat,
        theta: theta,
        scale: scale,
      };
      console.warn(move_graph_change);
      this.setState({ move_graph_change: move_graph_change });
    };

    let updateGraphPane = () => {
      let style = this.map.getPane("graph").style;
      /// transform origin
      let origin_p = this.map.latLngToLayerPoint(origin);
      style.transformOrigin = `${origin_p.x}px ${origin_p.y}px`;

      /// transform
      let trans_loc_p = this.map.latLngToLayerPoint(trans_loc);
      let rot_loc_p = this.map.latLngToLayerPoint(rot_loc);
      // Translation
      let xy_offs = trans_loc_p.subtract(origin_p); // x and y
      let x = xy_offs.x;
      let y = xy_offs.y;
      // Rotation
      let diff_p = rot_loc_p.subtract(trans_loc_p);
      let theta = Math.atan2(diff_p.x, diff_p.y);
      // Scale
      let scale = Math.sqrt(Math.pow(diff_p.x, 2) + Math.pow(diff_p.y, 2)) / unit_scale_p;
      //
      style.transform = `translate(${x}px, ${y}px) rotate(${(-theta / Math.PI) * 180}deg) scale(${scale}, ${scale})`;
    };

    //
    let updateTransMarker = (e) => {
      // Rotation marker moves with the translation marker.
      let trans_loc_p = this.map.latLngToLayerPoint(trans_loc);
      let rot_loc_p = this.map.latLngToLayerPoint(rot_loc);
      let diff_p = rot_loc_p.subtract(trans_loc_p);
      let new_trans_loc_p = this.map.latLngToLayerPoint(e.latlng);
      let new_rot_loc_p = new_trans_loc_p.add(diff_p);
      let new_rot_loc = this.map.layerPointToLatLng(new_rot_loc_p);
      this.rot_marker.setLatLng(new_rot_loc);
      // Scale marker also moves with the translation marker
      this.scale_marker.setLatLng(e.latlng);
      //
      trans_loc = e.latlng;
      rot_loc = new_rot_loc;
      //
      updateGraphPane();
    };
    // add translation marker to the map
    this.trans_marker.on("dragstart", () => this.map.scrollWheelZoom.disable());
    this.trans_marker.on("drag", updateTransMarker, this);
    this.trans_marker.on("dragend", () => {
      this.map.scrollWheelZoom.enable();
      computeMoveGraphUpdate();
    });
    this.trans_marker.addTo(this.map);

    let updateRotMarker = (e) => {
      // Adjust the size of the marker to connect the trans and rot marker.
      let trans_loc_p = this.map.latLngToLayerPoint(trans_loc);
      let rot_loc_p = this.map.latLngToLayerPoint(e.latlng);
      let diff_p = rot_loc_p.subtract(trans_loc_p);
      let scale_p = Math.sqrt(Math.pow(diff_p.x, 2) + Math.pow(diff_p.y, 2));
      let icon = this.scale_marker.options.icon;
      icon.options.iconSize = [2 * scale_p, 2 * scale_p];
      this.scale_marker.setIcon(icon);
      //
      rot_loc = e.latlng;
      //
      updateGraphPane();
    };
    // add rotation marker to the map
    this.rot_marker.on("dragstart", () => this.map.scrollWheelZoom.disable());
    this.rot_marker.on("drag", updateRotMarker, this);
    this.rot_marker.on("dragend", () => {
      this.map.scrollWheelZoom.enable();
      computeMoveGraphUpdate();
    });
    this.rot_marker.addTo(this.map);

    // add scale marker to the map
    this.scale_marker.addTo(this.map);

    // handle zoom
    let trans_rot_diff_p = null;
    this.moveGraphZoomStart = () => {
      // hide markers
      this.scale_marker.setOpacity(0);
      this.rot_marker.setOpacity(0);
      let trans_loc_p = this.map.latLngToLayerPoint(trans_loc);
      let rot_loc_p = this.map.latLngToLayerPoint(rot_loc);
      trans_rot_diff_p = rot_loc_p.subtract(trans_loc_p);
      // hide graph pane
      this.map.getPane("graph").style.zIndex = -100;
    };
    this.moveGraphZoomEnd = () => {
      // display markers
      this.scale_marker.setOpacity(MOVE_GRAPH_MARKER_OPACITY2);
      // Maintain the relative position of rotMarker and transMarker
      let trans_loc_p = this.map.latLngToLayerPoint(trans_loc);
      let new_rot_loc_p = trans_loc_p.add(trans_rot_diff_p);
      let new_rot_loc = this.map.layerPointToLatLng(new_rot_loc_p);
      this.rot_marker.setLatLng(new_rot_loc);
      this.rot_marker.setOpacity(MOVE_GRAPH_MARKER_OPACITY);
      // display graph pane
      this.map.getPane("graph").style.zIndex = 500;
      //
      rot_loc = new_rot_loc;
      //
      updateGraphPane();
    };
    //
    this.map.on("zoomstart", this.moveGraphZoomStart, this);
    this.map.on("zoomend", this.moveGraphZoomEnd, this);
  }

  finishMoveGraph() {
    console.info("Finish moving graph");
    //
    this.map.off("zoomstart", this.moveGraphZoomStart, this);
    this.map.off("zoomend", this.moveGraphZoomEnd, this);
    this.moveGraphZoomStart = null;
    this.moveGraphZoomEnd = null;
    //
    let style = this.map.getPane("graph").style;
    style.transformOrigin = null;
    style.transform = null;
    //
    this.trans_marker.remove();
    this.scale_marker.remove();
    this.rot_marker.remove();
    this.trans_marker = null;
    this.rot_marker = null;
    this.scale_marker = null;
  }

  startMoveRobot() {
    if (this.robot_marker.valid === false) return;
    console.info("Start moving robot");
    // remove current robot marker and replace it with a draggable marker
    this.robot_marker.marker.remove();
    //
    let curr_latlng = this.robot_marker.marker.getLatLng();
    // initialize vertex
    let closest_vertices = this.kdtree.nearest(curr_latlng, 1);
    let move_robot_vertex = closest_vertices ? closest_vertices[0][0] : { ...curr_latlng, id: -1 };
    this.setState({ move_robot_vertex: move_robot_vertex });
    //
    let interm_pos = null;
    let handleDrag = (e) => (interm_pos = e.latlng);
    let handleDragEnd = () => {
      let closest_vertices = this.kdtree.nearest(interm_pos, 1);
      let move_robot_vertex = closest_vertices ? closest_vertices[0][0] : this.move_robot_vertex;
      this.setState({ move_robot_vertex: move_robot_vertex });
      this.move_robot_marker.setLatLng(move_robot_vertex);
    };
    this.move_robot_marker = L.marker(curr_latlng, {
      draggable: true,
      icon: ROBOT_ICON,
      opacity: MOVE_ROBOT_OPACITY,
      pane: "graph",
      zIndexOffset: 100,
      rotationOrigin: "center",
      rotationAngle: 0,
    });
    this.move_robot_marker.on("drag", handleDrag);
    this.move_robot_marker.on("dragend", handleDragEnd);
    this.move_robot_marker.addTo(this.map);
  }

  finishMoveRobot() {
    if (this.robot_marker.valid === false) return;
    console.info("Finish moving robot");
    this.move_robot_marker.remove();
    this.move_robot_marker = null;
    this.setState({ move_robot_vertex: { lng: 0, lat: 0, id: -1 } });
    // add the robot marker back
    this.robot_marker.marker.addTo(this.map);
  }
}

export default GraphMap;
