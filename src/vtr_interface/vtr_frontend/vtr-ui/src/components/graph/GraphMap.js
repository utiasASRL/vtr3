import React from "react";
import {
  Map as LeafletMap,
  TileLayer,
  Polyline,
  ZoomControl,
} from "react-leaflet";
import RotatedMarker from "./RotatedMarker"; // react-leaflet does not have rotatable marker
import { icon } from "leaflet";
import "leaflet/dist/leaflet.css";

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
      graph_ready: false,
      // leaflet map
      current_location: { lat: 43.782, lng: -79.466 },
      lower_bound: { lat: 43.781596, lng: -79.467298 },
      upper_bound: { lat: 43.782806, lng: -79.464608 },
      // pose graph
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

  render() {
    const { current_location } = this.state;

    return (
      <LeafletMap
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
              key={idx} // \todo This is bad. Use a unique id for each path
              positions={coords}
              onClick={(e) => {
                // alert("clicked " + e);
                console.log("The path is clicked!");
                console.log(e);
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
        // graph
        points: i_map,
        paths: data.paths.map((p) => p.vertices),
        cycles: data.cycles.map((p) => p.vertices),
        branch: data.branch.vertices,
        junctions: data.junctions,
        root_id: data.root,
        // map
        current_location: data.mapCenter,
        lower_bound: data.minBnd,
        upper_bound: data.maxBnd,
        graph_ready: true,
        // \todo setup graph selectors
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
   * @param {Object} path    Change object representing the path of vertex ids
   * @param {Object} points  Change object representing the coordinates of the vertices
   * @return {Array} Array of vertex objects in order
   */
  _extractVertices(path, points) {
    let vertices = [];
    path.forEach((id) => vertices.push(points.get(id)));
    return vertices;
  }
}

export default GraphMap;
