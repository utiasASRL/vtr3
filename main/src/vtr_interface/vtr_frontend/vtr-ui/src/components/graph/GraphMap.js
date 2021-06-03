import "leaflet/dist/leaflet.css";
import shortid from "shortid";
import protobuf from "protobufjs";
import React from "react";
import L, { icon } from "leaflet";
import "leaflet-rotatedmarker";
import {
  Map as LeafletMap,
  Pane,
  Marker,
  TileLayer,
  Polyline,
  ZoomControl,
} from "react-leaflet";
import { kdTree } from "kd-tree-javascript";

import RotatedMarker from "./RotatedMarker"; // react-leaflet does not have rotatable marker
import robotIcon from "../../images/arrow.svg";
import targetIcon from "../../images/arrow-merge.svg";
import pathSvg from "../../images/path-icon.svg";
import pathSvg2 from "../../images/path-icon-2.svg";
import mergeCenterSvg from "../../images/merge-center.svg";
import mergeEndSvg from "../../images/merge-end.svg";
import mergeStartSvg from "../../images/merge-start.svg";
import moveMapTranslationSvg from "../../images/move-map-translation.svg";
import moveMapRotationSvg from "../../images/move-map-rotation.svg";

const pathIcon = new L.Icon({
  iconUrl: pathSvg,
  iconAnchor: [25, 50],
  iconSize: new L.Point(50, 50),
});
const pathIcon2 = new L.Icon({
  iconUrl: pathSvg2,
  iconAnchor: [25, 50],
  iconSize: new L.Point(50, 50),
});
const mergeCenterIcon = new L.Icon({
  iconUrl: mergeCenterSvg,
  iconSize: new L.Point(30, 30),
});
const mergeEndIcon = new L.Icon({
  iconUrl: mergeEndSvg,
  iconSize: new L.Point(40, 40),
});
const mergeStartIcon = new L.Icon({
  iconUrl: mergeStartSvg,
  iconSize: new L.Point(40, 40),
});
const moveMapTranslationIcon = new L.Icon({
  iconUrl: moveMapTranslationSvg,
  iconSize: new L.Point(40, 40),
});
const moveMapRotationIcon = new L.Icon({
  iconUrl: moveMapRotationSvg,
  iconSize: new L.Point(40, 40),
});

const poseGraphOpacity = 0.9;

/**
 * Performs a binary search on the host array. This method can either be
 * injected into Array.prototype or called with a specified scope like this:
 * binaryIndexOf.call(someArray, searchElement);
 *
 * @param {*} searchElement The item to search for within the array.
 * @return The index of the element which defaults to -1 when not found.
 */
function binaryIndexOf(searchElement) {
  var minIndex = 0;
  var maxIndex = this.length - 1;
  var currentIndex;
  var currentElement;
  // var resultIndex;

  while (minIndex <= maxIndex) {
    // resultIndex = currentIndex = ((minIndex + maxIndex) / 2) | 0;
    currentElement = this[currentIndex];
    if (currentElement < searchElement) minIndex = currentIndex + 1;
    else if (currentElement > searchElement) maxIndex = currentIndex - 1;
    else return currentIndex;
  }

  // Bitwise not ensures that arr.splice(Math.abs(arr.binaryIndexOf(e)), 0, elem)
  // inserts or overwrites the correct element
  return ~maxIndex;
}
// Monkey-patch the function into all Arrays
// eslint-disable-next-line
Array.prototype.binaryIndexOf = binaryIndexOf;

function rotate(r, theta) {
  var c = Math.cos(theta),
    s = Math.sin(theta);
  return { x: r.x * c - r.y * s, y: r.x * s + r.y * c };
}

function tfToGps(vertex, tf) {
  // Calculate the length of a degree of latitude and longitude in meters.
  var latToM =
    111132.92 +
    -559.82 * Math.cos(2 * vertex.lat) +
    1.175 * Math.cos(4 * vertex.lat) +
    -0.0023 * Math.cos(6 * vertex.lat);
  var lngToM =
    111412.84 * Math.cos(vertex.lat) +
    -93.5 * Math.cos(3 * vertex.lat) +
    0.118 * Math.cos(5 * vertex.lat);

  var dr = rotate(tf, vertex.theta);
  return { lat: dr.y / latToM + vertex.lat, lng: dr.x / lngToM + vertex.lng };
}

class GraphMap extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      // Leaflet map
      mapCenter: L.latLng(43.782, -79.466), // Home of VT&R!
      lowerBound: L.latLng(43.781596, -79.467298),
      upperBound: L.latLng(43.782806, -79.464608),
      // Pose graph
      graphLoaded: false, // Set to true on first load.
      graphReady: false, // Whether or not the graph is ready to be displayed.
      branch: [], // Vertices on the active branch.
      cycles: [], // All cycle elements. \todo Never used?
      currentPath: [], // Current path to repeat.
      junctions: [], // Junctions (deg{v} > 2)
      paths: [], // All path elements.
      rootId: -1,
      // Robot state
      robotReady: false,
      covRobotTarget: [],
      covRobotTrunk: [],
      robotLocation: L.latLng(0, 0), // Current location of the robot.
      robotOrientation: 0, // Current orientation of the robot.
      robotVertex: null, // The current closest vertex id to the robot.
      robotSeq: 0, // Current sequence along the path being followed (prevents path plotting behind the robot).
      targetLocation: L.latLng(0, 0), // Target location of the robot for merge.
      targetOrientation: 0, // Target orientation of the robot for merge.
      targetVertex: null, // Target vertex for merging.
      tRobotTarget: { x: 0, y: 0, theta: 0 }, // Desired merge pose of the robot.
      tRobotTrunk: { x: 0, y: 0, theta: 0 }, // Current pose of the robot.
      // Move graph
      moveMapOrigin: L.latLng(43.782, -79.466),
      moveMapPaths: [], // A copy of paths used for alignment.
      rotLoc: L.latLng(43.782, -79.466),
      transLoc: L.latLng(43.782, -79.466),
      zooming: false, // The alignment does not work very well with zooming.
      // Merge
      mergePath: [],
    };

    // Pose graph loading related.
    this.points = new Map(); // Mapping from VertexId to Vertex for path lookup.
    this.seq = 0;
    this.stamp = 0;
    this.tree = null; // A td-tree to find the nearest vertices.
    this.updates = [];
    this.proto = null;
    protobuf.load("/proto/graph.proto", (error, root) => {
      if (error) throw error;
      this.proto = root;
      this._loadGraph();
      this._loadInitRobotState();
    }); // Note: the callback is not called until the second time rendering.

    // Get the underlying leaflet map.
    this.map = null;
    this.setMap = (map) => {
      this.map = map.leafletElement;
      if (this.map) {
        this.map.on("zoomstart", this._onZoomStart, this);
        this.map.on("zoomend", this._onZoomEnd, this);
      }
    };
    // Markers used to move map.
    this.transMarker = null;
    this.rotMarker = null;
    this.unitScaleP = null; // Original scale of the graph == unitScaleP pixel distance between transMarker and rotMarker
    this.transRotDiffP = null; // Only used when zooming
    // Marker used to move robot and the target vertex to move to.
    this.robotMarker = null;
    this.robotVertex = null;
    // Markers used for merging
    this.mergeMarker = { s: null, c: null, e: null };
    this.mergeVertex = { s: null, c: null, e: null };
  }

  componentDidMount() {
    // Socket IO
    this.props.socket.on("robot/loc", this._loadRobotState.bind(this));
    this.props.socket.on("robot/path", this._loadCurrentPath.bind(this));
    this.props.socket.on("graph/update", this._loadGraphUpdate.bind(this));
  }

  componentDidUpdate(prevProps) {
    // Reload graph after reconnecting to SocketIO.
    if (!prevProps.socketConnected && this.props.socketConnected)
      this._loadGraph();
    // Tools: moveMap, moveRobot, merge, localize
    if (!prevProps.merge && this.props.merge) this._startMerge();
    if (!prevProps.moveMap && this.props.moveMap) this._startMoveMap();
    if (!prevProps.moveRobot && this.props.moveRobot) this._startMoveRobot();
    if (!prevProps.relocalize && this.props.relocalize) this._startRelocalize();
    if (prevProps.merge && !this.props.merge)
      this._finishMerge(this.props.userConfirmed);
    if (prevProps.moveMap && !this.props.moveMap)
      this._finishMoveMap(this.props.userConfirmed);
    if (prevProps.moveRobot && !this.props.moveRobot)
      this._finishMoveRobot(this.props.userConfirmed);
    if (prevProps.relocalize && !this.props.relocalize)
      this._finishRelocalize(this.props.userConfirmed);
  }

  componentWillUnmount() {
    // Socket IO
    this.props.socket.off("robot/loc", this._loadRobotState.bind(this));
    this.props.socket.off("robot/path", this._loadCurrentPath.bind(this));
    this.props.socket.off("graph/update", this._loadGraphUpdate.bind(this));
  }

  render() {
    const { addingGoalPath, addingGoalType, moveMap, selectedGoalPath } =
      this.props;
    const {
      branch,
      currentPath,
      graphReady,
      lowerBound,
      mapCenter,
      mergePath,
      moveMapPaths,
      paths,
      robotLocation,
      robotOrientation,
      robotReady,
      targetLocation,
      targetOrientation,
      targetVertex,
      upperBound,
      zooming,
    } = this.state;

    return (
      <LeafletMap
        ref={this.setMap}
        bounds={[
          [lowerBound.lat, lowerBound.lng],
          [upperBound.lat, upperBound.lng],
        ]}
        center={mapCenter}
        onClick={this._onMapClick.bind(this)}
        zoomControl={false}
      >
        {/* Google map tiles */}
        <TileLayer
          maxNativeZoom={20}
          maxZoom={22}
          noWrap
          subdomains="0123"
          // "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"  // default
          url="/cache/tile/{s}/{x}/{y}/{z}"
          // attribution="&copy; <a href=&quot;http://osm.org/copyright&quot;>OpenStreetMap</a> contributors"
        />
        <ZoomControl position="bottomright" />
        {/* Main graph and robot */}
        {graphReady && (
          <>
            {/* Current path (to repeat) */}
            <Pane
              style={{
                zIndex: 500, // \todo Magic number.
              }}
            >
              <Polyline
                color={"#f50057"}
                opacity={poseGraphOpacity}
                positions={this._extractVertices(currentPath, this.points).map(
                  (v) => [v.lat, v.lng]
                )}
                weight={5}
              />
            </Pane>
            {/* Current branch (during teach) */}
            <Pane
              style={{
                zIndex: 500, // \todo Magic number.
              }}
            >
              <Polyline
                color={"#f50057"}
                opacity={poseGraphOpacity}
                positions={this._extractVertices(branch, this.points).map(
                  (v) => [v.lat, v.lng]
                )}
                weight={5}
              />
            </Pane>
            {/* Graph paths */}
            {paths.map((path, idx) => {
              let vertices = this._extractVertices(path, this.points);
              let coords = vertices.map((v) => [v.lat, v.lng]);
              return (
                <Polyline
                  key={shortid.generate()}
                  color={"#ffc107"}
                  opacity={poseGraphOpacity}
                  positions={coords}
                  weight={7}
                />
              );
            })}
            {/* Current path considered for merging */}
            <Pane
              style={{
                zIndex: 500, // \todo Magic number.
              }}
            >
              <Polyline
                color={"#bfff00"}
                opacity={poseGraphOpacity}
                positions={this._extractVertices(mergePath, this.points).map(
                  (v) => [v.lat, v.lng]
                )}
                weight={5}
              />
            </Pane>
            {/* Robot marker */}
            {robotReady && (
              <RotatedMarker
                position={robotLocation}
                rotationAngle={robotOrientation}
                icon={icon({
                  iconUrl: robotIcon,
                  iconSize: [40, 40],
                })}
                opacity={0.85}
                zIndexOffset={20}
              />
            )}
            {/* Target robot marker for merging*/}
            {robotReady && targetVertex !== null && (
              <RotatedMarker
                position={targetLocation}
                rotationAngle={targetOrientation}
                icon={icon({
                  iconUrl: targetIcon,
                  iconSize: [40, 40],
                })}
                onClick={() => this._submitMerge()}
                opacity={0.85}
                zIndexOffset={30}
              />
            )}
            {/* Selected vertices for a repeat goal being added */}
            {addingGoalType === "Repeat" &&
              addingGoalPath.map((id, idx) => {
                if (!this.points.has(id)) return null;
                return (
                  <Marker
                    key={shortid.generate()}
                    position={this.points.get(id)}
                    icon={pathIcon}
                    opacity={0.8}
                  />
                );
              })}
            {/* Selected vertices for a repeat goal being selected (already added) */}
            {selectedGoalPath.map((id, idx) => {
              if (!this.points.has(id)) return null;
              return (
                <Marker
                  key={shortid.generate()}
                  position={this.points.get(id)}
                  icon={pathIcon2}
                  opacity={0.8}
                />
              );
            })}
          </>
        )}
        {/* A copy of the graph used for alignment */}
        {moveMap && !zooming && (
          <Pane // Change the transform style of pane to re-position the graph.
            style={{
              position: "absolute",
              top: "0",
              left: "0",
              transformOrigin: this._getTransformOriginString(),
              transform: this._getTransformString(),
            }}
          >
            {/* Graph paths */}
            {moveMapPaths.map((path, idx) => {
              let vertices = this._extractVertices(path, this.points);
              let coords = vertices.map((v) => [v.lat, v.lng]);
              return (
                <Polyline
                  color={"#ffc107"}
                  opacity={poseGraphOpacity}
                  key={shortid.generate()}
                  positions={coords}
                  weight={5}
                />
              );
            })}
          </Pane>
        )}
      </LeafletMap>
    );
  }

  /** @brief Loads the full pose graph via XMLHttpRequest. */
  _loadGraph() {
    console.log("Loading full pose graph.");
    let xhr = new XMLHttpRequest();
    xhr.open(
      /* method */ "GET",
      /* file */ "/api/map/" + this.seq,
      /* async */ true
    );
    xhr.responseType = "arraybuffer";
    xhr.onload = function () {
      if (this.proto === null) return;
      let data = this.proto
        .lookupType("Graph")
        .decode(new Uint8Array(xhr.response));
      this._updateFullGraphState(data);
    }.bind(this);
    xhr.send(null);
  }

  /**
   * @brief Loads intermediate updates of the pose graph.
   * @param {Object} data_proto Graph updates in ProtoBuf format.
   */
  _loadGraphUpdate(data_proto) {
    if (this.proto === null) return;
    // console.log("Loading pose graph updates.");
    let update = this.proto
      .lookupType("GraphUpdate")
      .decode(new Uint8Array(data_proto));
    // console.debug("[GraphMap] _loadGraphUpdate: update:", update);
    update.vertices.forEach((val) => {
      val.distanceTo = L.LatLng.prototype.distanceTo;
    });

    update.valueOf = () => update.seq;
    update.vertices.sort();

    if (update.invalidate) {
      this._loadGraph();
      return;
    }

    if (update.seq > this.seq) {
      // This is a new update.
      this.updates.push(update);
      this._applyGraphUpdate(update);
    } else if (this.updates.length > 0 && update.seq >= this.updates[0].seq) {
      // Don't consider updates that are really old
      var idx = this.updates.binaryIndexOf(update.seq);
      if (idx < 0) {
        // If we didn't have that update, insert it and replay everything since then
        idx = Math.abs(idx);
        this.updates.splice(idx, 0, update);
        this.updates.slice(idx).forEach(this._applyGraphUpdate(update), this);
      }
    }
  }

  /**
   * @brief Gets full graph state from a data object decoded from ProtoBuf.
   * @param {Object} data Full graph state.
   */
  _updateFullGraphState(data) {
    // initGraphFromMessage
    if (data.stamp > this.stamp) this.stamp = data.stamp;
    if (data.seq > this.seq) this.seq = data.seq;
    if (data.seq < 0) {
      console.log("Graph is in sync.");
      return;
    }
    console.log("Full graph at seq: ", this.seq, ", stamp:", this.stamp);

    // Remove any updates that are older than the current complete-graph message
    this.updates = this.updates
      .filter(function (val) {
        return val.seq > data.seq;
      })
      .sort(function (a, b) {
        if (a.seq < b.seq) return -1;
        else return Number(b.seq > a.seq);
      });

    this.points = new Map();
    data.vertices.forEach((val) => {
      val.valueOf = () => val.id;
      val.distanceTo = L.LatLng.prototype.distanceTo;
      val.weight = 0;
      this.points.set(val.id, val);
    });
    this.tree = new kdTree(data.vertices, (a, b) => b.distanceTo(a), [
      "lat",
      "lng",
    ]);

    // this.updates.forEach((v) => this._applyGraphUpdate(v));

    this.setState(
      (state, props) => {
        let initMapCenter = {};
        if (!state.graphLoaded)
          initMapCenter = {
            mapCenter: data.mapCenter,
            lowerBound: data.minBnd,
            upperBound: data.maxBnd,
          };
        return {
          // Map (only once)
          ...initMapCenter,
          // Graph
          graphLoaded: true, // One time state variable
          graphReady: true,
          paths: [
            ...data.paths.map((p) => p.vertices),
            ...data.cycles.map((p) => p.vertices),
          ],
          cycles: data.cycles.map((p) => p.vertices),
          branch: data.branch.vertices,
          junctions: data.junctions,
          rootId: data.root,
          // Copy of paths used for alignment
          moveMapPaths: [
            ...data.paths.map((p) => p.vertices),
            ...data.cycles.map((p) => p.vertices),
          ], // \todo Is it correct to put here?
        };
      },
      () => {
        // \todo This used to be put before branch, junctions, paths, etc are
        // updated, as shown above. Check if it matters.
        this.updates.forEach((v) => this._applyGraphUpdate(v));
        this._updateRobotState();
      }
    );
  }

  /**
   * @brief Applies intermediate update to the pose graph.
   * \todo This function has not been tested yet!
   * @param {Object} update Intermediate update to the pose graph.
   */
  _applyGraphUpdate(update) {
    this.setState((state) => {
      update.vertices.forEach((v) => {
        if (v.weight === undefined) v.weight = 0;
        this.points.set(v.id, v);
      });
      let lastId =
        state.branch.length > 0 ? state.branch[state.branch.length - 1] : 0;
      update.vertices.forEach((v) => {
        if (v.id > lastId) {
          this.tree.insert(v);
          state.branch.push(v.id);
        } else if (v.id < lastId) {
          let idx = state.branch.binaryIndexOf(v.id);
          if (idx < 0) {
            this.tree.insert(v);
            state.branch.splice(Math.abs(idx), 0, v.id);
          }
        }
      });
      return { branch: state.branch };
    }, this._updateRobotState.bind(this));
  }

  /** @brief Gets the initial robot state from json data. */
  _loadInitRobotState() {
    console.log("Loading initial robot state.");
    fetch("/api/init")
      .then((response) => {
        if (response.status !== 200) {
          console.error("Fetch initial robot state failed: " + response.status);
          return;
        }
        // Examine the text in the response
        response.json().then((data) => {
          this.setState(
            {
              robotReady: true,
              currentPath: data.path,
              robotVertex: data.vertex,
              robotSeq: data.seq,
              tRobotTrunk: {
                x: data.tfLeafTrunk[0],
                y: data.tfLeafTrunk[1],
                theta: data.tfLeafTrunk[2],
              },
              covRobotTrunk: data.covLeafTrunk,
              tRobotTarget: {
                x: data.tfLeafTarget[0],
                y: data.tfLeafTarget[1],
                theta: data.tfLeafTarget[2],
              },
              covRobotTarget: data.covLeafTarget,
            },
            this._updateRobotState.bind(this)
          );
        });
      })
      .catch((err) => {
        console.log("Fetch error: ", err);
      });
  }

  /** @brief Socket IO callback to update the robot state. */
  _loadRobotState(data_proto) {
    if (this.proto === null) return;
    let data = this.proto
      .lookupType("RobotStatus")
      .decode(new Uint8Array(data_proto));
    // console.debug("[GraphMap] _loadRobotState: data:", data);
    this.setState(
      {
        robotVertex: data.vertex,
        robotSeq: data.seq,
        tRobotTrunk: data.tfLeafTrunk,
        targetVertex: data.tfLeafTarget !== null ? data.targetVertex : null,
        tRobotTarget:
          data.tfLeafTarget !== null
            ? data.tfLeafTarget
            : { x: 0, y: 0, theta: 0 },
      },
      this._updateRobotState.bind(this)
    );
  }

  /**
   * @brief Updates the robot's location based on the current closest vertex id.
   */
  _updateRobotState() {
    this.setState((state) => {
      if (!state.graphLoaded) return;
      // Robot pose
      let loc = this.points.get(state.robotVertex);
      if (loc === undefined) return;
      let latlng = tfToGps(loc, state.tRobotTrunk);
      let theta = loc.theta - state.tRobotTrunk.theta;
      let robotPose = {
        robotLocation: latlng,
        robotOrientation: (-theta * 180) / Math.PI,
      };
      // Target pose
      if (state.targetVertex === null) return robotPose;
      loc = this.points.get(state.targetVertex);
      if (loc === undefined) return robotPose;
      latlng = tfToGps(loc, state.tRobotTarget);
      theta = loc.theta - state.tRobotTarget.theta;
      let targetPose = {
        targetLocation: latlng,
        targetOrientation: (-theta * 180) / Math.PI,
      };
      return { ...robotPose, ...targetPose };
    });
  }

  /**
   * @brief Loads the current path / localization chain.
   * @param {Object} data An object containing the current path.
   */
  _loadCurrentPath(data) {
    this.setState({ currentPath: data.path });
  }

  /**
   * @brief Extracts an array of vertex data from an Array of vertex IDs.
   * @param path   The path of vertex ids
   * @param points The coordinates of the vertices
   * @return Array of vertex objects in order
   */
  _extractVertices(path, points) {
    let vertices = [];
    path.forEach((id) => vertices.push(points.get(id)));
    return vertices;
  }

  /**
   * @brief Returns the closes vertex on graph according to user selected latlng
   * with some tolerance.
   * Helper function of _onMapClick.
   * @param {Object} latlng User selected lat and lng.
   * @param {number} tol Tolerance in terms of the window.
   */
  _getClosestPoint(latlng, tol = 0.02) {
    let bounds = this.map.getBounds();
    let maxDist = Math.max(
      bounds.getSouthWest().distanceTo(bounds.getNorthEast()) * tol,
      1
    );

    let res = this.tree.nearest(latlng, 1, maxDist);
    if (res.length > 0) return { target: res[0][0], distance: res[0][1] };
    else return { target: null, distance: maxDist };
  }

  /**
   * @brief Map click callback. Selects vertices if adding a repeat goal.
   * @param {Object} e Event object from clicking on the map.
   */
  _onMapClick(e) {
    this.setState((state, props) => {
      let best = this._getClosestPoint(e.latlng);
      if (best.target === null) return;
      if (props.addingGoalType !== "Repeat") return;
      props.setAddingGoalPath([...props.addingGoalPath, best.target.id]);
    });
  }

  /** Starts relocalization. Currently nothing to do here. */
  _startRelocalize() {
    console.debug("[GraphMap] _startRelocalize");
  }

  /**
   * @brief Sends loc_search command to main vtr process via Socket.IO if user
   * has confirmed.
   * @param {boolean} confirmed Whether user has confirmed the request.
   */
  _finishRelocalize(confirmed) {
    if (!confirmed) return;
    else {
      console.debug("[GraphMap] _finishRelocalize: confirmed relocalize.");
      this.setState(
        (state, props) => {
          props.socket.emit("graph/cmd", {
            action: "loc_search",
          });
        },
        () => {
          this.props.addressConf();
        }
      );
    }
  }

  /** Helper function to find user selected paths for merging.
   *
   * @param {Object} source Starting node.
   * @param {array} dests Array of destinations (start & end).
   */
  _breadthFirstSearch(source, dests) {
    let queue = [source];
    let done = new Set();

    let parents = new Map();
    parents.set(source, -1);

    let targets = new Set(dests);

    while (queue.length > 0 && targets.size > 0) {
      let node = queue.shift();
      if (done.has(node)) continue;
      done.add(node);
      this.points.get(node).neighbours.forEach((neighbour) => {
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

  /** Adds markers for merging to the map. */
  _startMerge() {
    console.debug("[GraphMap] _startMerge");
    // Initial position of the start, center and end markers based on current map.
    // Offset to start/end marker is ~10% of the screen diagonal in each direction.
    let mapBounds = this.map.getBounds();
    let sW = mapBounds.getSouthWest();
    let nE = mapBounds.getNorthEast();
    let offs = (Math.abs(sW.lat - nE.lat) + Math.abs(sW.lng - nE.lng)) / 20;
    let centerPos = this.map.getCenter();
    let startPos = {
      lat: centerPos.lat,
      lng: centerPos.lng - offs,
    };
    let endPos = {
      lat: centerPos.lat,
      lng: centerPos.lng + offs,
    };

    // Intermediate position of each maker that gets updated while dragging.
    let intermPos = { s: null, c: null, e: null };

    let getMergePath = () => {
      let paths = this._breadthFirstSearch(this.mergeVertex.c.id, [
        this.mergeVertex.s.id,
        this.mergeVertex.e.id,
      ]);
      paths[0].reverse();
      return paths[0].concat(paths[1].slice(1));
    };

    let getRotationAngle = (p1, p2) => {
      let point1 = this.map.project(this.points.get(p1));
      let point2 = this.map.project(this.points.get(p2));
      let diff = point2.subtract(point1);
      return 90 - (Math.atan2(diff.x, diff.y) * 180) / Math.PI;
    };

    // Drag handles for start and end markers.
    let handleDrag = (e, key) => (intermPos[key] = e.latlng);
    let handleDragEnd = (key) => {
      let closestVertices = this.tree.nearest(intermPos[key], 1);
      this.mergeVertex[key] = closestVertices
        ? closestVertices[0][0]
        : this.mergeVertex[key];
      // Find paths
      let mergePath = getMergePath();
      console.log("mergepath:", mergePath);

      this.setState({ mergePath: mergePath });
      // Set new marker locations
      let rotationAngle =
        mergePath.length < 2
          ? 0 // No enough vertices to calculate angle
          : key === "s"
          ? getRotationAngle(mergePath[1], mergePath[0])
          : getRotationAngle(
              mergePath[mergePath.length - 1],
              mergePath[mergePath.length - 2]
            );
      this.mergeMarker[key].setLatLng(this.mergeVertex[key]);
      this.mergeMarker[key].setRotationAngle(rotationAngle);
      intermPos[key] = null;
    };

    // Drag handles for the center marker. The start and end markers move with
    // it while it is dragging.
    let handleDragC = (e) => {
      intermPos.c = e.latlng;
      intermPos.s = {
        lat: this.mergeVertex.s.lat + e.latlng.lat - this.mergeVertex.c.lat,
        lng: this.mergeVertex.s.lng + e.latlng.lng - this.mergeVertex.c.lng,
      };
      this.mergeMarker.s.setLatLng(intermPos.s);
      intermPos.e = {
        lat: this.mergeVertex.e.lat + e.latlng.lat - this.mergeVertex.c.lat,
        lng: this.mergeVertex.e.lng + e.latlng.lng - this.mergeVertex.c.lng,
      };
      this.mergeMarker.e.setLatLng(intermPos.e);
    };
    let handleDragEndC = () => {
      let closestVertices = this.tree.nearest(intermPos.c, 1);
      this.mergeVertex.c = closestVertices
        ? closestVertices[0][0]
        : this.mergeVertex.c;
      this.mergeMarker.c.setLatLng(this.mergeVertex.c);
      intermPos.c = null;
      handleDragEnd("s");
      handleDragEnd("e");
    };
    // The center marker.
    let closestVertices = this.tree.nearest(centerPos, 1);
    this.mergeVertex.c = closestVertices ? closestVertices[0][0] : centerPos;
    this.mergeMarker.c = L.marker(this.mergeVertex.c, {
      draggable: true,
      zIndexOffset: 2000, // \todo Magic number.
      icon: mergeCenterIcon,
      opacity: poseGraphOpacity,
    });
    this.mergeMarker.c.on("drag", (e) => handleDragC(e));
    this.mergeMarker.c.on("dragend", () => handleDragEndC());
    this.mergeMarker.c.addTo(this.map);

    // Compute merge path here first for calculating rotation angle of start and
    // end marker.
    closestVertices = this.tree.nearest(startPos, 1);
    this.mergeVertex.s = closestVertices ? closestVertices[0][0] : startPos;
    closestVertices = this.tree.nearest(endPos, 1);
    this.mergeVertex.e = closestVertices ? closestVertices[0][0] : endPos;
    let mergePath = getMergePath();
    // The start marker
    this.mergeMarker.s = L.marker(this.mergeVertex.s, {
      draggable: true,
      zIndexOffset: 2000, // \todo Magic number.
      icon: mergeStartIcon,
      opacity: poseGraphOpacity,
      rotationOrigin: "center",
      rotationAngle:
        mergePath.length > 1 ? getRotationAngle(mergePath[1], mergePath[0]) : 0,
    });
    this.mergeMarker.s.on("drag", (e) => handleDrag(e, "s"));
    this.mergeMarker.s.on("dragend", () => handleDragEnd("s"));
    this.mergeMarker.s.addTo(this.map);
    // The end marker.
    this.mergeMarker.e = L.marker(this.mergeVertex.e, {
      draggable: true,
      zIndexOffset: 2000, // \todo Magic number.
      icon: mergeEndIcon,
      opacity: poseGraphOpacity,
      rotationOrigin: "center",
      rotationAngle:
        mergePath.length > 1
          ? getRotationAngle(
              mergePath[mergePath.length - 1],
              mergePath[mergePath.length - 2]
            )
          : 0,
    });
    this.mergeMarker.e.on("drag", (e) => handleDrag(e, "e"));
    this.mergeMarker.e.on("dragend", () => handleDragEnd("e"));
    this.mergeMarker.e.addTo(this.map);

    this.setState({ mergePath: mergePath });
  }

  /**
   * @brief Removes markers for merging and keep merge path if user has
   * confirmed it.
   * @param {boolean} confirmed Whether user has confirmed the change.
   */
  _finishMerge(confirmed) {
    let reset = () => {
      this.map.removeLayer(this.mergeMarker.c);
      this.map.removeLayer(this.mergeMarker.s);
      this.map.removeLayer(this.mergeMarker.e);
      this.mergeMarker = { s: null, c: null, e: null };
      this.mergeVertex = { s: null, c: null, e: null };
    };
    if (!confirmed) this.setState({ mergePath: [] }, reset());
    else {
      this.setState(
        (state, props) => {
          console.debug(
            "[GraphMap] _finishMerge: confirmed merge id:",
            this.mergeVertex.c.id,
            "path:",
            state.mergePath
          );
          props.socket.emit("graph/cmd", {
            action: "merge",
            vertex: this.mergeVertex.c.id,
            path: state.mergePath,
          });
        },
        () => {
          reset();
          this.props.addressConf();
        }
      );
    }
  }

  /**
   * @brief Target marker click callback. Sends merge command to main vtr
   * process.
   */
  _submitMerge() {
    console.debug("[GraphMap] _submitMerge");
    this.setState((state, props) => {
      /// TODO uncomment the following if necessary. Need some indication of
      /// whether we can close the loop
      // let cov = state.covRobotTarget;
      // let tf = state.tRobotTarget;
      // if (cov[0] > 0.25 || cov[1] > 0.1 || cov[2] > 0.1)
      //   console.error("Match covariance too high:", cov);
      // else if (tf.x > 0.5 || tf.y > 0.25 || tf.theta > 0.2)
      //   console.error("Offset too high:", tf);
      // else {
      console.log("Trying to merge at vertex", state.targetVertex);
      props.socket.emit("graph/cmd", { action: "closure" });
      return { mergePath: [] };
      // }
    });
  }

  /** @brief Adds the marker for moving the robot. */
  _startMoveRobot() {
    console.debug("[GraphMap] _startMoveRobot");
    let intermPos = null;
    let handleDrag = (e) => (intermPos = e.latlng);
    let handleDragEnd = () => {
      let closestVertices = this.tree.nearest(intermPos, 1);
      this.robotVertex = closestVertices
        ? closestVertices[0][0]
        : this.robotVertex;
      this.robotMarker.setLatLng(this.robotVertex);
    };
    this.setState(
      (state) => {
        let closestVertices = this.tree.nearest(state.robotLocation, 1);
        this.robotVertex = closestVertices
          ? closestVertices[0][0]
          : state.robotLocation;
        // Marker for translating the graph
        this.robotMarker = L.marker(this.robotVertex, {
          draggable: true,
          zIndexOffset: 2000, // \todo Magic number.
          icon: icon({
            iconUrl: robotIcon,
            iconSize: [40, 40],
          }),
          opacity: poseGraphOpacity,
          rotationAngle: state.robotOrientation,
          rotationOrigin: "center",
        });
        return { robotReady: false };
      },
      () => {
        this.robotMarker.on("drag", handleDrag);
        this.robotMarker.on("dragend", handleDragEnd);
        this.robotMarker.addTo(this.map);
      }
    );
  }

  /**
   * @brief Removes the marker for moving the robot and send changes via Socket
   * IO if user has confirmed.
   * @param {boolean} confirmed Whether user has confirmed the change.
   */
  _finishMoveRobot(confirmed) {
    let reset = () => {
      this.map.removeLayer(this.robotMarker);
      this.robotMarker = null;
      this.robotVertex = null;
      this.setState({ robotReady: true });
    };
    if (!confirmed) reset();
    else {
      console.debug("[GraphMap] _finishMoveRobot: confirmed robot location");
      this.setState(
        (state, props) => {
          props.socket.emit("graph/cmd", {
            action: "localize",
            vertex: this.robotVertex.id,
          });
        },
        () => {
          reset();
          this.props.addressConf();
        }
      );
    }
  }

  /** @brief Adds markers for translating and rotating the pose graph. */
  _startMoveMap() {
    console.debug("[GraphMap] _startMoveMap");
    this.setState(
      (state) => {
        let vid = state.rootId;
        let transLoc =
          vid >= 0 ? L.latLng(this.points.get(vid)) : L.latLng(0, 0);
        // Marker for translating the graph
        this.transMarker = L.marker(transLoc, {
          draggable: true,
          zIndexOffset: 2000, // \todo Magic number.
          icon: moveMapTranslationIcon,
          opacity: poseGraphOpacity,
        });

        let p_center = this.map.latLngToLayerPoint(transLoc);
        let p_bounds = this.map.getPixelBounds();
        this.unitScaleP =
          (p_bounds.max.x - p_bounds.min.x + p_bounds.max.y - p_bounds.min.y) /
          16.0;
        let rotLoc = this.map.layerPointToLatLng(
          p_center.add(L.point(0, this.unitScaleP))
        );
        // Marker for rotating the graph
        this.rotMarker = L.marker(rotLoc, {
          draggable: true,
          zIndexOffset: 3000, // \todo Magic number.
          icon: moveMapRotationIcon,
          opacity: poseGraphOpacity,
        });

        return {
          graphReady: false, // Make graph and robot invisible while moving. A frozen copy of it that will be displayed.
          moveMapOrigin: transLoc,
          transLoc: transLoc,
          rotLoc: rotLoc,
        };
      },
      () => {
        this.transMarker.on("dragstart", () =>
          this.map.scrollWheelZoom.disable()
        );
        this.transMarker.on("drag", this._updateTransMarker, this);
        this.transMarker.on("dragend", () => this.map.scrollWheelZoom.enable());
        this.transMarker.addTo(this.map);
        this.rotMarker.on("dragstart", () =>
          this.map.scrollWheelZoom.disable()
        );
        this.rotMarker.on("drag", this._updateRotMarker, this);
        this.rotMarker.on("dragend", () => this.map.scrollWheelZoom.enable());
        this.rotMarker.addTo(this.map);
      }
    );
  }

  /**
   * @brief Removes markers for translating and rotating the pose graph.
   * @param {boolean} confirmed Whether user has confirmed the change.
   */
  _finishMoveMap(confirmed) {
    let reset = () => {
      this.map.removeLayer(this.transMarker);
      this.map.removeLayer(this.rotMarker);
      this.transMarker = null;
      this.rotMarker = null;
      this.unitScaleP = null;
      this.setState({ graphReady: true });
    };
    if (!confirmed) reset();
    else {
      console.debug("[GraphMap] _finishMoveMap: Confirmed graph re-position.");
      this.setState(
        (state, props) => {
          let transLocP = this.map.latLngToLayerPoint(state.transLoc);
          let rotLocP = this.map.latLngToLayerPoint(state.rotLoc);
          let rotSub = rotLocP.subtract(transLocP);
          let theta = Math.atan2(rotSub.x, rotSub.y);
          let scale =
            Math.sqrt(Math.pow(rotSub.x, 2) + Math.pow(rotSub.y, 2)) /
            this.unitScaleP;
          let change = {
            x: state.transLoc.lng - state.moveMapOrigin.lng,
            y: state.transLoc.lat - state.moveMapOrigin.lat,
            theta: theta,
            scale: scale,
          };
          if (
            !(
              ((change.x === change.y) === change.theta) === 0 &&
              change.scale === 1
            ) // something changed
          )
            props.socket.emit("map/offset", change);
          return {
            moveMapOrigin: L.latLng(43.782, -79.466),
            transLoc: L.latLng(43.782, -79.466),
            rotLoc: L.latLng(43.782, -79.466),
          };
        },
        () => {
          reset();
          this.props.addressConf();
        }
      );
    }
  }

  /**
   * @brief Updates the current location of the transmarker in react state
   * variable and lets the rotmarker follow it.
   * @param {Object} e The event object from dragging the translation marker.
   */
  _updateTransMarker(e) {
    this.setState((state) => {
      // Rotation marker moves with the translation marker.
      let transLocP = this.map.latLngToLayerPoint(state.transLoc);
      let rotLocP = this.map.latLngToLayerPoint(state.rotLoc);
      let diff = rotLocP.subtract(transLocP);
      let newTransLocP = this.map.latLngToLayerPoint(e.latlng);
      let newRotLocP = newTransLocP.add(diff);
      let newRotLoc = this.map.layerPointToLatLng(newRotLocP);
      this.rotMarker.setLatLng(newRotLoc);
      return {
        transLoc: e.latlng,
        rotLoc: newRotLoc,
      };
    });
  }

  /**
   * @brief Updates the current location of the rotmarker in react state
   * variable in pixel coordinates.
   * @param {Object} e The event object from dragging the rotation marker.
   */
  _updateRotMarker(e) {
    this.setState({ rotLoc: e.latlng });
  }

  /**
   * @brief Hides rotMarker during zooming and records the relative pixel
   * distance between transMarker and rotMarker.
   */
  _onZoomStart() {
    this.setState((state) => {
      if (this.rotMarker) {
        // Remember the current position of rotMarker relative to transMarker so
        // that the pixel distance between the two do not change after zooming.
        this.rotMarker.setOpacity(0);
        let transLocP = this.map.latLngToLayerPoint(state.transLoc);
        let rotLocP = this.map.latLngToLayerPoint(state.rotLoc);
        this.transRotDiffP = rotLocP.subtract(transLocP);
      }
      return { zooming: true };
    });
  }

  /**
   * @brief Shows rotMarker upon zoom end and adjusts its position so that the
   * distance between transMarker and rotMarker in pixel remains unchanged.
   */
  _onZoomEnd() {
    this.setState((state) => {
      if (this.rotMarker) {
        // Maintain the relative position of rotMarker and transMarker
        let transLocP = this.map.latLngToLayerPoint(state.transLoc);
        let newRotLocP = transLocP.add(this.transRotDiffP);
        let newRotLoc = this.map.layerPointToLatLng(newRotLocP);
        this.rotMarker.setLatLng(newRotLoc);
        this.rotMarker.setOpacity(poseGraphOpacity);
        return { zooming: false, rotLoc: newRotLoc };
      } else {
        return { zooming: false };
      }
    });
  }

  /** @brief Returns the transform origin in pixel in current view. */
  _getTransformOriginString() {
    if (!this.map) return 0 + "px " + 0 + "px";
    let origin = this.map.latLngToLayerPoint(this.state.moveMapOrigin);
    return origin.x + "px " + origin.y + "px";
  }

  /**
   * @brief Returns the transformation based on current location of transmarker
   * and rotmarker in pixel coordinates.
   */
  _getTransform() {
    let originP = this.map.latLngToLayerPoint(this.state.moveMapOrigin);
    let transLocP = this.map.latLngToLayerPoint(this.state.transLoc);
    let rotLocP = this.map.latLngToLayerPoint(this.state.rotLoc);
    // Translation
    let xyOffs = transLocP.subtract(originP); // x and y
    // Rotation
    let rotSub = rotLocP.subtract(transLocP);
    let theta = Math.atan2(rotSub.x, rotSub.y);
    return { x: xyOffs.x, y: xyOffs.y, theta: theta };
  }

  /**
   * @brief Returns the css string of transformation based on current location
   * of transmarker and rotmarker in pixel coordinates.
   */
  _getTransformString() {
    if (!this.map) return "translate(" + 0 + "px, " + 0 + "px) ";
    let transform = this._getTransform();
    return (transform =
      "translate(" +
      transform.x +
      "px, " +
      transform.y +
      "px) rotate(" +
      (-transform.theta / Math.PI) * 180 +
      "deg)");
  }
}

export default GraphMap;
