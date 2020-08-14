import React from "react";
import { icon } from "leaflet";
import { Marker } from "react-leaflet";

import RotatedMarker from "./RotatedMarker"; // react-leaflet does not have rotatable marker
import markerIcon from "../../images/arrow-merge.svg";

class PointSelector extends React.Component {
  constructor(props) {
    super(props);

    // array[vertices][vertex,distance]
    let closestVertices = props.tree.nearest(props.initLocation, 1);
    this.state = {
      // A vertex contains latlng so we can pass it to marker position directly
      markerVertex: closestVertices
        ? closestVertices[0][0]
        : props.initLocation,
    };

    this.intermLocation = null;
  }

  render() {
    const { markerVertex } = this.state;
    return (
      <Marker
        draggable={true}
        position={markerVertex}
        icon={icon({
          iconUrl: markerIcon,
          iconSize: [30, 30],
        })}
        ondrag={(e) => (this.intermLocation = e.latlng)}
        ondragend={this._handleDragEnd.bind(this)}
        opacity={0.85}
        zIndexOffset={20}
      />
    );
  }

  _handleDragEnd() {
    let vertex = null;
    this.setState(
      (state, props) => {
        let closestVertices = props.tree.nearest(this.intermLocation, 1);
        vertex = closestVertices ? closestVertices[0][0] : state.markerVertex;
        return {
          markerVertex: vertex,
        };
      },
      () => this.props.setSelectedVertices([vertex])
    );
  }
}

export { PointSelector };
