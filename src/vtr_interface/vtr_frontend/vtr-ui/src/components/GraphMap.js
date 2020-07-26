import React from "react";
import { Map as LeafletMap, TileLayer, Polyline } from "react-leaflet";
import "leaflet/dist/leaflet.css";

import protobuf from "protobufjs";

class GraphMap extends React.Component {
  constructor(props) {
    super(props);

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
    };

    // \todo pose graph loading related
    this.seq = 0;
    this.stamp = 0;
    this.updates = [];
    this.tree = null; // a td-tree to find nearest neighbor

    protobuf.load("/proto/Graph.proto", (error, root) => {
      if (error) throw error;
      this.proto = root;
      this._reloadGraph();
    }); // note: the callback is not called until the second time render.
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
      let graph_proto = this.proto.lookupType("Graph");
      let graph_msg = graph_proto.decode(new Uint8Array(xhr.response));
      // alert(JSON.stringify(msg, null, 4));  // verify correctly decoded
      this._graphLoaded(graph_msg);
    }.bind(this);
    xhr.send(null);
  }

  _graphLoaded(graph_msg) {
    console.log(graph_msg);

    // initGraphFromMessage
    if (graph_msg.stamp > this.stamp) this.stamp = graph_msg.stamp;
    if (graph_msg.seq > this.seq) this.seq = graph_msg.seq;

    if (graph_msg.seq < 0) {
      // \todo check: seq > 0 if the graph is newer?
      console.log("Graph is in sync.");
      return;
    }

    console.log("Initialized to seq: ", this.seq, ", stamp:", this.stamp);

    // Remove any updates that are older than the current complete-graph message
    this.updates = this.updates
      .filter(function (val) {
        return val.seq > graph_msg.seq;
      })
      .sort(function (a, b) {
        if (a.seq < b.seq) return -1;
        else return Number(b.seq > a.seq);
      });

    var i_map = new Map();
    // \todo are the conversions still necessary?
    graph_msg.vertices.forEach((val) => {
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
        paths: graph_msg.paths.map((p) => p.vertices),
        cycles: graph_msg.cycles.map((p) => p.vertices),
        branch: graph_msg.branch.vertices,
        junctions: graph_msg.junctions,
        root_id: graph_msg.root,
      };
    });

    // \todo need to check graph_ready?
    this.setState((state, props) => {
      return {
        // map
        current_location: graph_msg.mapCenter,
        lower_bound: graph_msg.minBnd,
        upper_bound: graph_msg.maxBnd,
        graph_ready: true,
        // \todo setup graph selectors
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
