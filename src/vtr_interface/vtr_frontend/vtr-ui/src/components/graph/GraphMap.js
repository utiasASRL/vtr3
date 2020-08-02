import React from "react";
import {
  Map as LeafletMap,
  Pane,
  Marker,
  TileLayer,
  Polyline,
  ZoomControl,
} from "react-leaflet";
import RotatedMarker from "./RotatedMarker"; // react-leaflet does not have rotatable marker
import L, { icon } from "leaflet";
import "leaflet/dist/leaflet.css";
import shortid from "shortid";
import protobuf from "protobufjs";
import { kdTree } from "kd-tree-javascript";

import robotIcon from "../../images/arrow.svg";

const alignMarkerOpacity = 0.8;

/**
 * Performs a binary search on the host array. This method can either be
 * injected into Array.prototype or called with a specified scope like this:
 * binaryIndexOf.call(someArray, searchElement);
 *
 * @param searchElement The item to search for within the array.
 * @return The index of the element which defaults to -1 when not found.
 */
function binaryIndexOf(searchElement) {
  var minIndex = 0;
  var maxIndex = this.length - 1;
  var currentIndex;
  var currentElement;
  var resultIndex;

  while (minIndex <= maxIndex) {
    resultIndex = currentIndex = ((minIndex + maxIndex) / 2) | 0;
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
Array.prototype.binaryIndexOf = binaryIndexOf;

function rotate(r, theta) {
  var c = Math.cos(theta),
    s = Math.sin(theta);
  return { x: r.x * c - r.y * s, y: r.x * s + r.y * c };
}

function tfToGps(vertex, tf) {
  // Calculate the length of a degree of latitude and longitude in meters
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
      // leaflet map
      mapCenter: L.latLng(43.782, -79.466), // Home of VT&R!
      lowerBound: L.latLng(43.781596, -79.467298),
      upperBound: L.latLng(43.782806, -79.464608),
      // pose graph
      graphLoaded: false, // set to true on first load
      graphReady: false, // whether or not the graph is ready to be displayed.
      points: new Map(), // mapping from VertexId to Vertex for path lookup
      branch: [], // vertices on the active branch
      junctions: [], // all junctions (deg{v} > 2)
      paths: [], // all path elements
      cycles: [], // all cycle elements \todo bug in the original node, never used?
      rootId: -1,
      // robot state
      currentPath: [],
      robotLocation: L.latLng(0, 0), // Current location of the robot
      robotOrientation: 1.6,
      robotVertex: 0, // The current closest vertex id to the robot
      robotSeq: 0, // Current sequence along the path being followed (prevents path plotting behind the robot)
      tRobotTrunk: { x: 0, y: 0, theta: 0 },
      covRobotTrunk: [],
      tRobotTarget: { x: 0, y: 0, theta: 0 },
      covRobotTarget: [],
      // alignment tool
      zooming: false, // the alignment does not work very well with zooming.
      alignOrigin: L.latLng(43.782, -79.466),
      transLoc: L.latLng(43.782, -79.466),
      rotLoc: L.latLng(43.782, -79.466),
      alignPaths: [], // A copy of paths used by alignment.
    };

    // Pose graph loading related. \todo tree updates not used for now.
    this.seq = 0;
    this.stamp = 0;
    this.updates = [];
    this.tree = null; // a td-tree to find nearest neighbor
    this.proto = null;
    protobuf.load("/proto/Graph.proto", (error, root) => {
      if (error) throw error;
      this.proto = root;
      this._loadGraph();
      this._loadInitRobotState();
    }); // note: the callback is not called until the second time render.

    // Get the underlying leaflet map. Needed by the alignment tool.
    this.map = null;
    this.setMap = (map) => {
      this.map = map.leafletElement;
      if (this.map) {
        this.map.on("zoomstart", this._onZoomStart, this);
        this.map.on("zoomend", this._onZoomEnd, this);
      }
    };
    this.transMarker = null;
    this.rotMarker = null;
    this.unitScaleP = null; // Original scale of the graph == unitScaleP pixel distance between transMarker and rotMarker
    this.transRotDiffP = null; // Only used when zooming
  }

  componentDidMount() {
    this.props.socket.on("robot/loc", this._loadRobotState.bind(this));
    this.props.socket.on("graph/update", this._loadGraphUpdate.bind(this));
  }

  componentDidUpdate(prevProps) {
    // Reload graph after reconnecting to SocketIO.
    if (!prevProps.socketConnected && this.props.socketConnected)
      this._loadGraph();
    // Alignment markers are not parts of react components. They are added
    // through reaflet API directly, so we need to update them manually here.
    if (!prevProps.pinMap && this.props.pinMap) this._startPinMap();
    if (prevProps.pinMap && !this.props.pinMap)
      this._finishPinMap(this.props.userConfirmed);
    if (this.props.userConfirmed) this.props.addressConf();
  }

  render() {
    const { addingGoalType, addingGoalPath } = this.props;
    const { mapCenter, lowerBound, upperBound, points } = this.state;

    return (
      <LeafletMap
        ref={this.setMap}
        center={mapCenter}
        bounds={[
          [lowerBound.lat, lowerBound.lng],
          [upperBound.lat, upperBound.lng],
        ]}
        zoomControl={false}
        onClick={this._onMapClick.bind(this)}
      >
        {/* Google map tiles */}
        <TileLayer
          // "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"  // default
          url="/cache/tile/{s}/{x}/{y}/{z}"
          maxNativeZoom={20}
          maxZoom={22}
          subdomains="0123"
          noWrap
          // attribution="&copy; <a href=&quot;http://osm.org/copyright&quot;>OpenStreetMap</a> contributors"
        />
        <ZoomControl position="bottomright" />
        {/* Main graph and robot */}
        {this.state.graphReady && (
          <>
            {/* Graph paths */}
            {this.state.paths.map((path, idx) => {
              let vertices = this._extractVertices(path, points);
              let coords = vertices.map((v) => [v.lat, v.lng]);
              return <Polyline key={shortid.generate()} positions={coords} />;
            })}
            {/* Robot marker */}
            <RotatedMarker
              position={this.state.robotLocation}
              rotationAngle={this.state.robotOrientation}
              icon={icon({
                iconUrl: robotIcon,
                iconSize: [40, 40],
              })}
              opacity={0.85}
            />
            {/* Selected goals for a repeat goal to be added */}
            {addingGoalType === "Repeat" &&
              addingGoalPath.map((id, idx) => {
                if (!points.has(id)) return;
                return (
                  <Marker
                    key={shortid.generate()}
                    position={points.get(id)}
                    icon={icon({
                      iconUrl: robotIcon,
                      iconSize: [40, 40],
                    })}
                    opacity={0.4}
                  />
                );
              })}
          </>
        )}
        {/* A copy of the map used for alignment */}
        {this.props.pinMap && !this.state.zooming && (
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
            {this.state.alignPaths.map((path, idx) => {
              let vertices = this._extractVertices(path, points);
              let coords = vertices.map((v) => [v.lat, v.lng]);
              return <Polyline key={shortid.generate()} positions={coords} />;
            })}
          </Pane>
        )}
      </LeafletMap>
    );
  }

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
      // alert(JSON.stringify(msg, null, 4));  // verify correctly decoded
      this._updateFullGraphState(data);
    }.bind(this);
    xhr.send(null);
  }

  _loadGraphUpdate(data_proto) {
    if (this.proto === null) return;
    console.log("Loading pose graph updates.");
    let update = this.proto
      .lookupType("GraphUpdate")
      .decode(new Uint8Array(data_proto));
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
      // This is a new update
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

  /** Used to be graphLoaded
   *
   * Get graph state from proto data.
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

    var iMap = new Map();
    // \todo are the conversions still necessary?
    data.vertices.forEach((val) => {
      val.valueOf = () => val.id;
      val.distanceTo = L.LatLng.prototype.distanceTo;
      val.weight = 0;
      iMap.set(val.id, val);
    });
    // \todo construct kd-tree
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
          graphLoaded: true, // one-time state variable
          graphReady: true,
          points: iMap,
          paths: data.paths.map((p) => p.vertices),
          cycles: data.cycles.map((p) => p.vertices),
          branch: data.branch.vertices,
          junctions: data.junctions,
          rootId: data.root,
          // Copy of paths used for alignment
          alignPaths: data.paths.map((p) => p.vertices), // \todo correct to put here?
        };
      },
      () => {
        // \todo This used to be put before branch, junctions, paths, etc are
        // updated. Check if this matters.
        this.updates.forEach((v) => this._applyGraphUpdate(v));
        this._updateRobotState();
      }
    );
  }

  /** This graph has not been tested yet! */
  _applyGraphUpdate(update) {
    this.setState((state) => {
      update.vertices.forEach((v) => {
        if (v.weight === undefined) v.weight = 0;
        state.points.set(v.id, v);
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
      return { points: state.points, branch: state.branch };
    }, this._updateRobotState.bind(this));
  }

  /** Gets the initial robot state from json data.
   */
  _loadInitRobotState() {
    console.log("Loading initial robot state.");
    fetch("/api/init")
      .then((response) => {
        if (response.status !== 200) {
          console.log("Fetch initial robot state failed: " + response.status);
          return;
        }
        // Examine the text in the response
        response.json().then((data) => {
          this.setState(
            {
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
            this._updateRobotState.bind(this) // call here to guarantee state update.
          );
        });
      })
      .catch((err) => {
        console.log("Fetch error: ", err);
      });
  }

  /** Socket IO callback to update the robot state at real time.
   *
   * Used to be _robotCallback.
   */
  _loadRobotState(data_proto) {
    if (this.proto === null) return;
    let data = this.proto
      .lookupType("RobotStatus")
      .decode(new Uint8Array(data_proto));
    this.setState(
      {
        robotVertex: data.vertex,
        robotSeq: data.seq,
        tRobotTrunk: data.tfLeafTrunk,
        // \todo Add target_vertex and tRobotTarget
      },
      this._updateRobotState.bind(this)
    );
  }

  /** Used to be _robotChanged
   *
   * Updates the robot's location based on the current closest vertex id
   */
  _updateRobotState() {
    this.setState((state, prop) => {
      if (!state.graphLoaded) return;

      let loc = state.points.get(state.robotVertex);
      if (loc === undefined) return;

      // \todo (old) actually apply the transform
      //        var latlng = this._applyTf(loc, tRobotTrunk.base);
      //        var latlng = loc;
      let latlng = tfToGps(loc, state.tRobotTrunk);
      let theta = loc.theta - state.tRobotTrunk.theta;

      return {
        robotLocation: latlng,
        robotOrientation: (-theta * 180) / Math.PI,
      };
    });
  }

  /**
   * Extracts an array of vertex data from an Array of vertex IDs
   *
   * @param path   The path of vertex ids
   * @param points The coordinates of the vertices
   * @return Array of vertex objects in order
   */
  _extractVertices(path, points) {
    let vertices = [];
    path.forEach((id) => vertices.push(points.get(id)));
    return vertices;
  }

  /** Returns the closes vertex on graph according to user selected latlng with
   * some tolerance.
   *
   * Helper function of _onMapClick.
   *
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

  /** Map click callback. Selects vertices if adding a repeat goal. */
  _onMapClick(e) {
    let best = this._getClosestPoint(e.latlng);
    if (best.target === null) return;
    this.setState((state, props) => {
      if (props.addingGoalType !== "Repeat") return;
      props.setAddingGoalPath([...props.addingGoalPath, best.target.id]);
    });
  }

  /** Adds markers for translating and rotating the pose graph.
   */
  _startPinMap() {
    this.setState(
      (state) => {
        let vid = state.rootId;
        let transLoc =
          vid >= 0 ? L.latLng(state.points.get(vid)) : L.latLng(0, 0);
        // Marker for translating the graph
        this.transMarker = L.marker(transLoc, {
          draggable: true,
          zIndexOffset: 20,
          icon: icon({
            iconUrl: robotIcon,
            iconSize: [40, 40],
          }),
          opacity: alignMarkerOpacity,
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
          zIndexOffset: 30,
          icon: icon({
            iconUrl: robotIcon,
            iconSize: [40, 40],
          }),
          opacity: alignMarkerOpacity,
        });

        return {
          graphReady: false, // Make graph and robot invisible while moving. A frozen copy of it that will be displayed.
          alignOrigin: transLoc,
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

  /** Removes markers for translating and rotating the pose graph.
   */
  _finishPinMap(confirmed) {
    let resetAlign = () => {
      this.map.removeLayer(this.transMarker);
      this.map.removeLayer(this.rotMarker);
      this.transMarker = null;
      this.rotMarker = null;
      this.unitScaleP = null;
      this.setState({ graphReady: true });
    };
    if (!confirmed) {
      resetAlign();
    } else {
      console.log("[GraphMap] _finishedPinMap: Confirmed graph re-position.");
      this.setState((state, props) => {
        let transLocP = this.map.latLngToLayerPoint(state.transLoc);
        let rotLocP = this.map.latLngToLayerPoint(state.rotLoc);
        let rotSub = rotLocP.subtract(transLocP);
        let theta = Math.atan2(rotSub.x, rotSub.y);
        let scale =
          Math.sqrt(Math.pow(rotSub.x, 2) + Math.pow(rotSub.y, 2)) /
          this.unitScaleP;
        let change = {
          x: state.transLoc.lng - state.alignOrigin.lng,
          y: state.transLoc.lat - state.alignOrigin.lat,
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
          alignOrigin: L.latLng(43.782, -79.466),
          transLoc: L.latLng(43.782, -79.466),
          rotLoc: L.latLng(43.782, -79.466),
        };
      }, resetAlign);
    }
  }

  /** Updates the current location of the transmarker in react state variable,
   * and lets the rotmarker follow it.
   *
   * Used to be the _drag function.
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

  /** Updates the current location of the rotmarker in react state variable in
   * pixel coordinates. */
  _updateRotMarker(e) {
    this.setState({ rotLoc: e.latlng });
  }

  /** Hides rotmarker during zooming and keeps the relative pixel distance
   * between transMarker and rotMarker.
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
  _onZoomEnd() {
    this.setState((state) => {
      if (this.rotMarker) {
        // Maintain the relative position of rotMarker and transMarker
        let transLocP = this.map.latLngToLayerPoint(state.transLoc);
        let newRotLocP = transLocP.add(this.transRotDiffP);
        let newRotLoc = this.map.layerPointToLatLng(newRotLocP);
        this.rotMarker.setLatLng(newRotLoc);
        this.rotMarker.setOpacity(alignMarkerOpacity);
        return { zooming: false, rotLoc: newRotLoc };
      } else {
        return { zooming: false };
      }
    });
  }

  /** Returns the transform origin in pixel in current view. */
  _getTransformOriginString() {
    if (!this.map) return 0 + "px " + 0 + "px";
    let origin = this.map.latLngToLayerPoint(this.state.alignOrigin);
    return origin.x + "px " + origin.y + "px";
  }

  /** Returns the transform based on current location of transmarker and
   * rotmarker in pixel coordinates.
   */
  _getTransform() {
    let originP = this.map.latLngToLayerPoint(this.state.alignOrigin);
    let transLocP = this.map.latLngToLayerPoint(this.state.transLoc);
    let rotLocP = this.map.latLngToLayerPoint(this.state.rotLoc);
    // Translation
    let xyOffs = transLocP.subtract(originP); // x and y
    // Rotation
    let rotSub = rotLocP.subtract(transLocP);
    let theta = Math.atan2(rotSub.x, rotSub.y);
    return { x: xyOffs.x, y: xyOffs.y, theta: theta };
  }
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
