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
import { MapContainer, TileLayer } from "react-leaflet";
import { kdTree } from "kd-tree-javascript";

import ToolsMenu from "../tools/ToolsMenu";

import MergeCenterSVG from "../../images/merge-center.svg";
import MergeEndSVG from "../../images/merge-end.svg";
import MergeStartSVG from "../../images/merge-start.svg";

/// pose graph constants
const ROUTE_TYPE_COLOR = ["#f44336", "#ff9800", "#ffeb3b", "#4caf50", "#00bcd4", "#2196f3", "#9c27b0"];
const GRAPH_OPACITY = 0.9;
const GRAPH_WEIGHT = 7;

/// selector constants
const MERGE_CENTER_ICON = new L.Icon({
  iconUrl: MergeCenterSVG,
  iconSize: new L.Point(30, 30),
});
const MERGE_END_ICON = new L.Icon({
  iconUrl: MergeEndSVG,
  iconSize: new L.Point(40, 40),
});
const MERGE_START_ICON = new L.Icon({
  iconUrl: MergeStartSVG,
  iconSize: new L.Point(40, 40),
});

class GraphMap extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      /// tools menu
      current_tool: null,
      // annotate route
      annotate_route_type: 0,
      annotate_route_ids: [],
    };

    /// leaflet map
    this.map = null; // leaflet map instance

    /// pose graph states
    this.id2vertex = null; // id2vertex map
    this.kdtree = null; // kdtree for fast nearest neighbor search (created on demand)
    this.fixed_routes = [];
    this.active_routes = [];
    this.current_route = null;
  }

  componentDidMount() {
    // Socket IO
    this.props.socket.on("graph/update", this.graphUpdateCallback.bind(this));
  }

  componentWillUnmount() {
    // Socket IO
    this.props.socket.off("graph/update", this.graphUpdateCallback.bind(this));
  }

  render() {
    const { socket } = this.props;
    const { current_tool, annotate_route_type, annotate_route_ids } = this.state;

    return (
      <>
        {/* Leaflet map container with initial center set to UTIAS (only for initialization) */}
        <MapContainer center={[43.782, -79.466]} zoom={13} whenCreated={this.mapCreatedCallback.bind(this)}>
          {/* Leaflet map background */}
          <TileLayer
            // url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" // load from OSM directly
            url="/tile/{s}/{x}/{y}/{z}" // load from backend (potentially cached)
            maxZoom={20}
          />
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
        />
      </>
    );
  }

  /** @brief Leaflet map creationg callback */
  mapCreatedCallback(map) {
    console.debug("Leaflet map created.");
    //
    this.map = map;
    //
    this.fetchGraphState(true);
    //
    this.fetchRobotState(true);
  }

  fetchGraphState(center = false) {
    console.info("Fetching the current pose graph state (full).");
    fetch("/vtr/graph")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch pose graph state: " + response.status);
        response.json().then((data) => {
          console.info("Received the pose graph state (full).");
          console.info(data);
          this.loadGraphState(data, center);
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  /** @brief Helper function to convert a pose graph route to a leaflet polyline, and add it to map */
  route2Polyline(route) {
    // fixed_routes format: [{type: 0, ids: [id, ...]}, ...]
    let color = ROUTE_TYPE_COLOR[route.type % ROUTE_TYPE_COLOR.length];
    let latlngs = route.ids.map((id) => {
      let v = this.id2vertex.get(id);
      return [v.lat, v.lng];
    });
    let polyline = L.polyline(latlngs, { color: color, opacity: GRAPH_OPACITY, weight: GRAPH_WEIGHT });
    polyline.addTo(this.map);
    return polyline;
  }

  /** @brief Refresh the pose graph completely */
  loadGraphState(graph, center = false) {
    console.info("Loading the current pose graph state (full).");
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
    // current route
    if (this.current_route !== null) this.current_route.polyline.remove();
    if (graph.current_route.ids.length === 0) this.current_route = null;
    else this.current_route = { ...graph.current_route, polyline: this.route2Polyline(graph.current_route) };

    // center set to vertex (0, 0)
    if (center && this.id2vertex.get(0) !== undefined) {
      let v = this.id2vertex.get(0);
      this.map.panTo([v.lat, v.lng]);
    }
  }

  fetchRobotState(center = false) {
    console.info("Fetching the current robot state (full).");
    fetch("/vtr/robot")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch robot state: " + response.status);
        response.json().then((data) => {
          console.info("Received the robot state (full): ", data);
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  graphUpdateCallback(graph_update) {
    console.info("Received graph update: ", graph_update);
    if (this.map === null) return;

    // id2vertex and kdtree update
    graph_update.vertices.forEach((v) => {
      v.valueOf = () => v.id;
      v.distanceTo = L.LatLng.prototype.distanceTo;
      // only update if the vertex is not in the map (vertex position does not change)
      if (!this.id2vertex.has(v.id)) this.kdtree.insert(v);
      // always update the vertex map, because vertex neighbors may change
      this.id2vertex.set(v.id, v);
    });

    // current route and active routes updates
    if (this.current_route === null) {
      console.info("Starting a new route of type: ", graph_update.type);
      let current_route_info = {
        type: graph_update.type,
        ids: graph_update.vertices.map((v) => v.id),
      };
      this.current_route = { ...current_route_info, polyline: this.route2Polyline(current_route_info) };
    } else {
      if (this.current_route.type === graph_update.type) {
        console.info("Extending the current route of type: ", graph_update.type);
        let last_id = this.current_route.ids[this.current_route.ids.length - 1];
        graph_update.vertices.forEach((v) => {
          // current route ids must be incremental (usually there should be only two vertices in the update message)
          if (v.id > last_id) {
            this.current_route.ids.push(v.id);
            this.current_route.polyline.addLatLng([v.lat, v.lng]);
          }
        });
      } else {
        console.info("Extending the current route with a new type: ", graph_update.type);
        this.active_routes.push(this.current_route);
        let current_route_info = {
          type: graph_update.type,
          ids: graph_update.vertices.map((v) => v.id),
        };
        this.current_route = { ...current_route_info, polyline: this.route2Polyline(current_route_info) };
      }
    }
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
        }
      }
      // select the new tool
      switch (tool) {
        case "annotate_route":
          this.startAnnotateRoute();
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
      this.id2vertex.get(node).neighbours.forEach((neighbour) => {
        if (!parents.has(neighbour)) parents.set(neighbour, node);
        targets.delete(neighbour);
        queue.push(neighbour);
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
      icon: MERGE_CENTER_ICON,
      opacity: GRAPH_OPACITY,
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
      icon: MERGE_START_ICON,
      opacity: GRAPH_OPACITY,
      rotationOrigin: "center",
      rotationAngle: selected_path.length > 1 ? getRotationAngle(selected_path[1], selected_path[0]) : 0,
    });
    selector.marker.s.on("drag", (e) => handleDrag(e, "s"));
    selector.marker.s.on("dragend", () => handleDragEnd("s"));
    selector.marker.s.addTo(this.map);
    // The end marker.
    selector.marker.e = L.marker(selector.vertex.e, {
      draggable: true,
      icon: MERGE_END_ICON,
      opacity: GRAPH_OPACITY,
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

  startAnnotateRoute() {
    console.info("Start annotate route");
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
    console.info("Finish annotate route");
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
}

export default GraphMap;
