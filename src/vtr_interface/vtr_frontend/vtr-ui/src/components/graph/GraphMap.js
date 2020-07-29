import React from "react";
import {
  Map as LeafletMap,
  Pane,
  TileLayer,
  Polyline,
  ZoomControl,
} from "react-leaflet";
import RotatedMarker from "./RotatedMarker"; // react-leaflet does not have rotatable marker
import L, { icon } from "leaflet";
import "leaflet/dist/leaflet.css";

import shortid from "shortid";

import protobuf from "protobufjs";

import robotIcon from "../../images/arrow.svg";

const alignMarkerOpacity = 0.8;

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

    // \todo pose graph loading related
    this.seq = 0;
    this.stamp = 0;
    this.updates = [];
    this.tree = null; // a td-tree to find nearest neighbor

    this.state = {
      // leaflet map
      currentLocation: { lat: 43.782, lng: -79.466 }, // Home of VT&R!
      lowerBound: { lat: 43.781596, lng: -79.467298 },
      upperBound: { lat: 43.782806, lng: -79.464608 },
      // pose graph
      graphReady: false,
      points: new Map(), // mapping from VertexId to Vertex for path lookup
      branch: [], // vertices on the active branch
      junctions: [], // all junctions (deg{v} > 2)
      paths: [], // all path elements
      cycles: [], // all cycle elements \todo bug in the original node, never used?
      rootId: -1,
      // robot state
      currentPath: [],
      robotLocation: { lat: 0, lng: 0 }, // Current location of the robot
      robotOrientation: 1.6,
      robotVertex: 0, // The current closest vertex id to the robot
      robotSeq: 0, // Current sequence along the path being followed (prevents path plotting behind the robot)
      tRobotTrunk: { x: 0, y: 0, theta: 0 },
      covRobotTrunk: [],
      tRobotTarget: { x: 0, y: 0, theta: 0 },
      covRobotTarget: [],
      // alignment tool
      zooming: false, // the alignment does not work very well with zooming.
      alignOrigin: { lat: 43.782, lng: -79.466 },
      transLoc: { lat: 43.782, lng: -79.466 },
      rotLoc: { lat: 43.782, lng: -79.466 },
      alignPaths: [], // A copy of paths used by alignment.
    };

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

    protobuf.load("/proto/Graph.proto", (error, root) => {
      if (error) throw error;
      this.proto = root;
      this._reloadGraph();
      this._loadInitRobotState();
    }); // note: the callback is not called until the second time render.
  }

  componentDidMount() {
    this.props.socket.on("robot/loc", this._loadRobotState.bind(this));
  }

  componentDidUpdate(prevProps) {
    // Alignment markers are not parts of react components. They are added
    // through reaflet API directly, so we need to update them manually here.
    if (!prevProps.pinMap && this.props.pinMap) this._addTransRotMarkers();
    if (prevProps.pinMap && !this.props.pinMap) this._removeTransRotMarkers();
  }

  render() {
    const { currentLocation } = this.state;

    return (
      <LeafletMap
        ref={this.setMap}
        center={currentLocation}
        bounds={[
          [this.state.lowerBound.lat, this.state.lowerBound.lng],
          [this.state.upperBound.lat, this.state.upperBound.lng],
        ]}
        zoomControl={false}
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
        {/* Graph paths */}
        {this.state.paths.map((path, idx) => {
          let vertices = this._extractVertices(path, this.state.points);
          let coords = vertices.map((v) => [v.lat, v.lng]);
          return (
            <Polyline
              key={shortid.generate()}
              positions={coords}
              onClick={(e) => {
                // alert("clicked " + e);
                console.log("The path is clicked!");
              }}
            />
          );
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
              let vertices = this._extractVertices(path, this.state.points);
              let coords = vertices.map((v) => [v.lat, v.lng]);
              return (
                <Polyline
                  key={shortid.generate()}
                  positions={coords}
                  onClick={(e) => {
                    // alert("clicked " + e);
                    console.log("The path is clicked!");
                  }}
                />
              );
            })}
          </Pane>
        )}
      </LeafletMap>
    );
  }

  _reloadGraph() {
    console.log("Trying to fetch graph");
    let xhr = new XMLHttpRequest();
    xhr.open(
      /* method */ "GET",
      /* file */ "/api/map/" + this.seq,
      /* async */ true
    );
    xhr.responseType = "arraybuffer";
    xhr.onload = function () {
      let data = this.proto
        .lookupType("Graph")
        .decode(new Uint8Array(xhr.response));
      // alert(JSON.stringify(msg, null, 4));  // verify correctly decoded
      this._updateGraphState(data);
    }.bind(this);
    xhr.send(null);
  }

  /** Used to be graphLoaded
   *
   * Get graph state from proto data.
   */
  _updateGraphState(data) {
    // initGraphFromMessage
    if (data.stamp > this.stamp) this.stamp = data.stamp;
    if (data.seq > this.seq) this.seq = data.seq;

    if (data.seq < 0) {
      // \todo check: seq > 0 if the graph is newer?
      console.log("Graph is in sync.");
      return;
    }

    console.log("Initialized to seq: ", this.seq, ", stamp:", this.stamp);

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
      val.weight = 0;
      iMap.set(val.id, val);
    });
    // \todo construct kd-tree
    // \todo apply any updates that came in after the map message was produced.
    // \todo call _robotChanged()

    this.setState((state, props) => {
      return {
        // Map
        currentLocation: data.mapCenter,
        lowerBound: data.minBnd,
        upperBound: data.maxBnd,
        // Graph
        points: iMap,
        paths: data.paths.map((p) => p.vertices),
        cycles: data.cycles.map((p) => p.vertices),
        branch: data.branch.vertices,
        junctions: data.junctions,
        rootId: data.root,
        graphReady: true,
        // Copy of paths used for alignment
        alignPaths: data.paths.map((p) => p.vertices), // \todo correct to put here?
      };
    }, this._updateRobotState.bind(this));
  }

  /** Gets the initial robot state from json data.
   */
  _loadInitRobotState() {
    console.log("Trying to fetch initial robot state.");
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

  /** Socket IO callback to update the robot state at real time. */
  _loadRobotState(data_proto) {
    if (this.proto === undefined) return;
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
   *
   */
  _updateRobotState() {
    this.setState((state, prop) => {
      if (!state.graphReady) return;

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

  /** Adds markers for translating and rotating the pose graph.
   */
  _addTransRotMarkers() {
    this.setState(
      (state) => {
        let vid = state.rootId;
        let transLoc = vid >= 0 ? this.state.points.get(vid) : L.latLng(0, 0);
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
  _removeTransRotMarkers() {
    this.map.removeLayer(this.transMarker);
    this.map.removeLayer(this.rotMarker);
    this.transMarker = null;
    this.rotMarker = null;
    this.unitScaleP = null;
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
