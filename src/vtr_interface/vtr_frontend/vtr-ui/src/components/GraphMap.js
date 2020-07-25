import React from 'react';
import { Map, TileLayer } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';

import protobuf from "protobufjs"

import graph_proto from "../proto/Graph.proto"

class GraphMap extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      graph_ready: false,
      current_location: { lat: 43.782, lng: -79.466 },
      lower_bound: { lat: 43.781596, lng: -79.467298 },
      upper_bound: { lat: 43.782806, lng: -79.464608 },
    }

    this.seq = 0; // \todo graph seq

    protobuf.load(graph_proto, (error, root) => {
      if (error) throw error;
      this.proto = root;
      this._reloadGraph();
    });  // note: the callback is not called until the second time render.
  }

  render() {
    const { current_location } = this.state;

    return (
      <Map
        center={current_location}
        bounds={[
          [this.state.lower_bound.lat, this.state.lower_bound.lng],
          [this.state.upper_bound.lat, this.state.upper_bound.lng]]}
      >
        <TileLayer
          // "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"  // default
          url='/cache/tile/{s}/{x}/{y}/{z}'
          maxNativeZoom={20}
          maxZoom={22}
          subdomains='0123'
          noWrap
        // attribution="&copy; <a href=&quot;http://osm.org/copyright&quot;>OpenStreetMap</a> contributors"
        />
      </Map>
    );
  }

  _reloadGraph() {
    console.log("Trying to fetch graph");
    let xhr = new XMLHttpRequest();
    xhr.open(
      /* method */ 'GET',
      /* file */   '/api/map/' + this.seq,
      /* async */  true,
    );
    xhr.responseType = 'arraybuffer';
    xhr.onload = (function () {
      let graph_proto = this.proto.lookupType("Graph");
      let graph_msg = graph_proto.decode(new Uint8Array(xhr.response));
      // alert(JSON.stringify(msg, null, 4));  // verify correctly decoded
      this._graphLoaded(graph_msg);
    }).bind(this);
    xhr.send(null);
  }

  _graphLoaded(graph_msg) {
    // \todo need to check graph_ready?
    this.setState((state, props) => {
      return {
        current_location: graph_msg.mapCenter,
        lower_bound: graph_msg.minBnd,
        upper_bound: graph_msg.maxBnd,
      }
    })
  }
}

export default GraphMap;
