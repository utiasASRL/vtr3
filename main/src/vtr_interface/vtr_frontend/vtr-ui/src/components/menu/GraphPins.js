import shortid from "shortid";
import React from "react";

import Paper from "@material-ui/core/Paper";
import Button from "@material-ui/core/Button";
import CheckIcon from "@material-ui/icons/Check";
import ClearIcon from "@material-ui/icons/Clear";
import IconButton from "@material-ui/core/IconButton";
import Table from "@material-ui/core/Table";
import TableBody from "@material-ui/core/TableBody";
import TableCell from "@material-ui/core/TableCell";
import TableContainer from "@material-ui/core/TableContainer";
import TableHead from "@material-ui/core/TableHead";
import TableRow from "@material-ui/core/TableRow";
import TextField from "@material-ui/core/TextField";
import { withStyles } from "@material-ui/core/styles";

const styles = (theme) => ({
  graphPinsTable: {
    position: "absolute",
    top: 150,
    right: 5,
    height: 300,
    width: 500,
    zIndex: 2000, // \todo This is a magic number.
  },
});

class GraphPins extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      weight: -1, // user selected weight for the current pin, -1 means infinity since weight can only be positive.
      weightStr: "",
      pinsSnapshot: [], // create a snap shot of the pins in case user does not confirm so that we revert to the previous state.
    };
  }

  componentDidUpdate(prevProps) {
    if (!prevProps.pinGraph && this.props.pinGraph) {
      // create a snapshot of the current array of pins
      this.setState((state, props) => {
        return { pinsSnapshot: [...props.graphPins] };
      });
    }
    if (prevProps.pinGraph && !this.props.pinGraph) {
      let reset = () => {
        // reset everything when we exit this mode
        this.setState((state, props) => {
          props.resetGraphPin();
          return { weight: -1, weightStr: "", pinsSnapshot: [] };
        });
      };
      if (!this.props.userConfirmed) {
        console.debug("[GraphMap] componentDidUpdate: revert graph pins.");
        this.setState(
          (state, props) => {
            // revert the pins state if user cancels
            props.setGraphPins(state.pinsSnapshot);
          },
          () => reset()
        );
      } else {
        console.debug("[GraphMap] componentDidUpdate: Confirmed graph pins.");
        this.setState(
          (state, props) => {
            // send our update to backend, and vtr will return the updated pins
            props.socket.emit("graph/pins", { pins: props.graphPins });
          },
          () => {
            reset();
            this.props.addressConf();
          }
        );
      }
    }
  }

  render() {
    const {
      classes,
      graphPins,
      graphPinType,
      graphPinLatLng,
      graphPinVertex,
      pinGraph,
      removeGraphPin,
      setGraphPinType,
    } = this.props;
    const { weight, weightStr } = this.state;
    return (
      <>
        {pinGraph && (
          <TableContainer component={Paper} className={classes.graphPinsTable}>
            <Table aria-label="graph pins table" size="small">
              <TableHead>
                <TableRow>
                  <TableCell align="center" style={{ width: "30%" }}>
                    VERTEX ID
                  </TableCell>
                  <TableCell align="center" style={{ width: "30%" }}>
                    LOCATION
                  </TableCell>
                  <TableCell align="center" style={{ width: "30%" }}>
                    WEIGHT
                  </TableCell>
                  <TableCell
                    align="center"
                    style={{ width: "10%" }}
                  ></TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {/* List of pins already added */}
                {graphPins.map((pin, index) => {
                  return (
                    <TableRow key={shortid.generate()}>
                      <TableCell align="center">
                        {this._vertexToText(pin.id)}
                      </TableCell>
                      <TableCell align="center">
                        {this._latLngToText(pin.latLng)}
                      </TableCell>
                      <TableCell align="center">
                        {this._weightToText(pin.weight)}
                      </TableCell>
                      <TableCell align="center">
                        <IconButton
                          disabled={index === 0} // first pin corresponds to root and cannot be removed
                          color="secondary"
                          onClick={() => {
                            removeGraphPin(index);
                          }}
                          size="small"
                        >
                          <ClearIcon />
                        </IconButton>
                      </TableCell>
                    </TableRow>
                  );
                })}
                {/* new pin */}
                <TableRow>
                  <TableCell align="center">
                    <Button
                      color={
                        graphPinType === "vertex" ? "secondary" : "primary"
                      }
                      disableElevation={true}
                      fullWidth={true}
                      onClick={() => {
                        if (graphPinType === "vertex") setGraphPinType(null);
                        else setGraphPinType("vertex");
                      }}
                      size="small"
                      variant={"contained"}
                    >
                      {this._vertexToText(graphPinVertex)}
                    </Button>
                  </TableCell>
                  <TableCell align="center">
                    <Button
                      color={
                        graphPinType === "latlng" ? "secondary" : "primary"
                      }
                      disableElevation={true}
                      fullWidth={true}
                      onClick={() => {
                        if (graphPinType === "latlng") setGraphPinType(null);
                        else setGraphPinType("latlng");
                      }}
                      size="small"
                      variant={"contained"}
                    >
                      {this._latLngToText(graphPinLatLng)}
                    </Button>
                  </TableCell>
                  <TableCell align="center">
                    <TextField
                      id="standard-basic"
                      label="WEIGHT"
                      onChange={(e) =>
                        this.setState({ weightStr: e.target.value })
                      }
                      onKeyPress={this._setWeight.bind(this)}
                      size="small"
                      value={weightStr}
                    />
                  </TableCell>
                  <TableCell align="center">
                    <IconButton
                      color="secondary"
                      onClick={() => this._generateAndAddPin()}
                      size="small"
                    >
                      <CheckIcon />
                    </IconButton>
                  </TableCell>
                </TableRow>
              </TableBody>
            </Table>
          </TableContainer>
        )}
      </>
    );
  }

  /** @brief */
  _generateAndAddPin() {
    this.setState((state, props) => {
      console.debug("[GraphMap] _generateAndAddPin: current ", props.graphPins);
      if (props.graphPinVertex === null || props.graphPinLatLng === null) {
        console.debug(
          "[GraphMap] _generateAndAddPin: vertex or latlng not selected"
        );
        return;
      }
      let newPin = {
        id: props.graphPinVertex,
        latLng: props.graphPinLatLng,
        weight: state.weight,
      };
      // reset pin type, vertex and latlng
      props.addGraphPin(newPin);
    });
  }

  _removePin(index) {
    this.setState((state, props) => {
      console.log("Trying to remove pin with id: ", index);
    });
  }

  /** @brief Create a string from vertex id. */
  _vertexToText(v) {
    if (v === null) {
      return "MAJ-MIN";
    } else {
      let vl = parseInt(v % Math.pow(2, 32));
      let vh = parseInt((v - vl) / Math.pow(2, 32));
      let s = vh.toString() + "-" + vl.toString();
      return s;
    }
  }

  /** @brief Create a string from lat lng. */
  _latLngToText(latLng) {
    if (latLng === null) {
      return "(LNG, LAT)";
    } else {
      return "(" + latLng.lng.toFixed(4) + "," + latLng.lat.toFixed(4) + ")";
    }
  }

  _weightToText(weight) {
    return weight < 0 ? "Inf" : weight.toFixed(1);
  }

  _setWeight(e) {
    if (e.key === "Enter") {
      let input = e.target.value;
      if (isNaN(input)) {
        this.setState({ weight: -1.0, weightStr: "" });
      } else {
        console.log("Weight parsed to be: ", parseFloat(input));
        this.setState({ weight: parseFloat(input) });
      }
    }
  }
}

export default withStyles(styles)(GraphPins);
