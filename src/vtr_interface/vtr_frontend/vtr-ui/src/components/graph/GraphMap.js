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

import robot_icon from "../../images/arrow.svg";

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
      current_location: { lat: 43.782, lng: -79.466 }, // Home of VT&R!
      lower_bound: { lat: 43.781596, lng: -79.467298 },
      upper_bound: { lat: 43.782806, lng: -79.464608 },
      zooming: false, // the alignment does not work very well with zooming.
      // pose graph
      graph_ready: false,
      points: new Map(), // mapping from VertexId to Vertex for path lookup
      branch: [], // vertices on the active branch
      junctions: [], // all junctions (deg{v} > 2)
      paths: [], // all path elements
      cycles: [], // all cycle elements \todo bug in the original node, never used?
      root_id: -1,
      // robot state
      current_path: [],
      robot_location: { lat: 0, lng: 0 }, // Current location of the robot
      robot_orientation: 1.6,
      robot_vertex: 0, // The current closest vertex id to the robot
      robot_seq: 0, // Current sequence along the path being followed (prevents path plotting behind the robot)
      t_robot_trunk: { x: 0, y: 0, theta: 0 },
      cov_robot_trunk: [],
      t_robot_target: { x: 0, y: 0, theta: 0 },
      cov_robot_target: [],
      // alignment tool
      align_origin: { lat: 43.782, lng: -79.466 },
      trans_loc: { lat: 43.782, lng: -79.466 },
      rot_loc: { lat: 43.782, lng: -79.466 },
      align_paths: [], // A copy of paths used for alignment.
    };

    // Get the underlying leaflet map. Needed for the alignment tool.
    this.map = null;
    this.setMap = (map) => {
      this.map = map.leafletElement;
      if (this.map) {
        this.map.on("zoomstart", () => this.setState({ zooming: true }), this);
        this.map.on("zoomend", () => this.setState({ zooming: false }), this);
      }
    };

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

  componentDidUpdate(prev_props) {
    // Alignment markers are not parts of react components. They are added
    // through reaflet API directly, so we need to update them manually here.
    if (!prev_props.pinMap && this.props.pinMap) this._addTransRotMarkers();
    if (prev_props.pinMap && !this.props.pinMap) this._removeTransRotMarkers();
  }

  render() {
    const { current_location } = this.state;

    return (
      <LeafletMap
        ref={this.setMap}
        center={current_location}
        bounds={[
          [this.state.lower_bound.lat, this.state.lower_bound.lng],
          [this.state.upper_bound.lat, this.state.upper_bound.lng],
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
          position={this.state.robot_location}
          rotationAngle={this.state.robot_orientation}
          icon={icon({
            iconUrl: robot_icon,
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
              transformOrigin: this._getTransformOrigin(),
              transform: this._getTransform(),
            }}
          >
            {/* Graph paths */}
            {this.state.align_paths.map((path, idx) => {
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

    var i_map = new Map();
    // \todo are the conversions still necessary?
    data.vertices.forEach((val) => {
      val.valueOf = () => val.id;
      val.weight = 0;
      i_map.set(val.id, val);
    });
    // \todo construct kd-tree
    // \todo apply any updates that came in after the map message was produced.
    // \todo call _robotChanged()

    this.setState((state, props) => {
      return {
        // Map
        current_location: data.mapCenter,
        lower_bound: data.minBnd,
        upper_bound: data.maxBnd,
        // Graph
        points: i_map,
        paths: data.paths.map((p) => p.vertices),
        cycles: data.cycles.map((p) => p.vertices),
        branch: data.branch.vertices,
        junctions: data.junctions,
        root_id: data.root,
        graph_ready: true,
        // Copy of paths used for alignment
        align_paths: data.paths.map((p) => p.vertices), // \todo correct to put here?
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
              current_path: data.path,
              robot_vertex: data.vertex,
              robot_seq: data.seq,
              t_robot_trunk: {
                x: data.tfLeafTrunk[0],
                y: data.tfLeafTrunk[1],
                theta: data.tfLeafTrunk[2],
              },
              cov_robot_trunk: data.covLeafTrunk,
              t_robot_target: {
                x: data.tfLeafTarget[0],
                y: data.tfLeafTarget[1],
                theta: data.tfLeafTarget[2],
              },
              cov_robot_target: data.covLeafTarget,
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
        robot_vertex: data.vertex,
        robot_seq: data.seq,
        t_robot_trunk: data.tfLeafTrunk,
        // \todo Add target_vertex and t_robot_target
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
      if (!state.graph_ready) return;

      let loc = state.points.get(state.robot_vertex);
      if (loc === undefined) return;

      // \todo (old) actually apply the transform
      //        var latlng = this._applyTf(loc, tRobotTrunk.base);
      //        var latlng = loc;
      let latlng = tfToGps(loc, state.t_robot_trunk);
      let theta = loc.theta - state.t_robot_trunk.theta;

      return {
        robot_location: latlng,
        robot_orientation: (-theta * 180) / Math.PI,
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
        let vid = state.root_id;
        let trans_loc = vid >= 0 ? this.state.points.get(vid) : L.latLng(0, 0);
        // Marker for translating the graph
        this.trans_marker = L.marker(trans_loc, {
          draggable: true,
          zIndexOffset: 20,
          icon: icon({
            iconUrl: robot_icon,
            iconSize: [40, 40],
          }),
          opacity: 0.8,
        });

        let p_center = this.map.latLngToLayerPoint(trans_loc);
        let p_bounds = this.map.getPixelBounds();
        let r0 =
          (p_bounds.max.x - p_bounds.min.x + p_bounds.max.y - p_bounds.min.y) /
          16.0;
        let rot_loc = this.map.layerPointToLatLng(p_center.add(L.point(0, r0)));
        // Marker for rotating the graph
        this.rot_marker = L.marker(rot_loc, {
          draggable: true,
          zIndexOffset: 30,
          icon: icon({
            iconUrl: robot_icon,
            iconSize: [40, 40],
          }),
          opacity: 0.8,
        });

        return {
          align_origin: trans_loc,
          trans_loc: trans_loc,
          rot_loc: rot_loc,
        };
      },
      () => {
        this.trans_marker.on("drag", this._updateTransMarker, this);
        this.trans_marker.addTo(this.map);
        this.rot_marker.on("drag", this._updateRotMarker, this);
        this.rot_marker.addTo(this.map);
      }
    );
  }

  /** Removes markers for translating and rotating the pose graph.
   */
  _removeTransRotMarkers() {
    this.map.removeLayer(this.trans_marker);
    this.map.removeLayer(this.rot_marker);
  }

  /** Updates the current location of the transmarker in react state variable,
   * and lets the rotmarker follow it.
   *
   * Used to be the _drag function.
   */
  _updateTransMarker(e) {
    this.setState((state) => {
      // Rotation marker moves with the translation marker.
      let trans_loc_p = this.map.latLngToLayerPoint(state.trans_loc);
      let rot_loc_p = this.map.latLngToLayerPoint(state.rot_loc);
      let diff = rot_loc_p.subtract(trans_loc_p);
      let new_trans_loc_p = this.map.latLngToLayerPoint(e.latlng);
      let new_rot_loc_p = new_trans_loc_p.add(diff);
      let new_rot_loc = this.map.layerPointToLatLng(new_rot_loc_p);
      this.rot_marker.setLatLng(new_rot_loc);
      return {
        trans_loc: e.latlng,
        rot_loc: new_rot_loc,
      };
    });
  }

  /** Updates the current location of the rotmarker in react state variable in
   * pixel coordinates. */
  _updateRotMarker(e) {
    this.setState({ rot_loc: e.latlng });
  }

  /** Returns the transform origin in pixel in current view. */
  _getTransformOrigin() {
    if (!this.map) return 0 + "px " + 0 + "px";
    let origin = this.map.latLngToLayerPoint(this.state.align_origin);
    return origin.x + "px " + origin.y + "px";
  }

  /** Returns the transform based on current location of transmarker and
   * rotmarker in pixel coordinates.
   */
  _getTransform() {
    if (!this.map) return "translate(" + 0 + "px, " + 0 + "px) ";
    let origin_p = this.map.latLngToLayerPoint(this.state.align_origin);
    let trans_loc_p = this.map.latLngToLayerPoint(this.state.trans_loc);
    let rot_loc_p = this.map.latLngToLayerPoint(this.state.rot_loc);
    // Translation
    let xy_offs = trans_loc_p.subtract(origin_p); // x and y
    // Rotation
    let rot_sub = rot_loc_p.subtract(trans_loc_p);
    let theta = Math.atan2(rot_sub.x, rot_sub.y);
    let transform =
      "translate(" +
      xy_offs.x +
      "px, " +
      xy_offs.y +
      "px) rotate(" +
      (-theta / Math.PI) * 180 +
      "deg)";
    return transform;
  }
}

export default GraphMap;
