import io from "socket.io-client";
import React from "react";
import ReactDOM from "react-dom";

import "fontsource-roboto"; // for Material UI library, the font package
import { withStyles } from "@material-ui/core/styles";

import "./index.css";
import GraphMap from "./components/graph/GraphMap";
import GoalManager from "./components/goal/GoalManager";
import ToolsMenu from "./components/menu/Toolsmenu";
import GraphPins from "./components/menu/GraphPins";

// SocketIO port is assumed to be UI port+1: (Number(window.location.port) + 1)
const socket = io(window.location.hostname + ":5201");

// Style
const styles = (theme) => ({
  vtrUI: {
    height: "100%",
    position: "absolute",
    width: "100%",
  },
  graphMap: {
    height: "100%",
    position: "absolute",
    width: "100%",
    zIndex: 0,
  },
});

class VTRUI extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      // Socket IO
      socketConnected: false,
      // Tools menu
      toolsState: {
        merge: false,
        relocalize: false,
        moveRobot: false,
        moveMap: false,
        pinGraph: false,
      },
      currTool: null,
      userConfirmed: false,
      // Goal manager
      addingGoalType: "Idle",
      addingGoalPath: [],
      selectedGoalPath: [],
      mergePath: [],
      // Graph Pins
      graphPins: [], // array of pins current set by user
      graphPinType: null, // the user is choosing "vertex" or choosing "latlng"
      graphPinLatLng: null, // current lat lng selected by the user
      graphPinVertex: null, // current vertex selected by the user
    };
  }

  componentDidMount() {
    console.debug("[index] componentDidMount: VTRUI mounted.");
    // Socket IO
    socket.on("connect", this._handleSocketConnect.bind(this, true));
    socket.on("disconnect", this._handleSocketConnect.bind(this, false));
  }

  componentWillUnmount() {
    // Socket IO
    socket.off("connect", this._handleSocketConnect.bind(this, true));
    socket.off("disconnect", this._handleSocketConnect.bind(this, false));
  }

  render() {
    const { classes } = this.props;
    const {
      addingGoalPath,
      addingGoalType,
      selectedGoalPath,
      mergePath,
      socketConnected,
      // Tools Menu
      toolsState,
      userConfirmed,
      // Graph Pins
      graphPins,
      graphPinType,
      graphPinLatLng,
      graphPinVertex,
    } = this.state;
    return (
      <div className={classes.vtrUI}>
        <GoalManager
          // Socket IO
          socket={socket}
          socketConnected={socketConnected}
          // Select path for repeat
          addingGoalPath={addingGoalPath}
          addingGoalType={addingGoalType}
          selectedGoalPath={selectedGoalPath}
          setAddingGoalPath={this._setAddingGoalPath.bind(this)}
          setAddingGoalType={this._setAddingGoalType.bind(this)}
          setSelectedGoalPath={this._setSelectedGoalPath.bind(this)}
          setMergePath={this._setMergePath.bind(this)}
          // Tools for merge and localize
          requireConf={this._requireConfirmation.bind(this)}
          selectTool={this._selectTool.bind(this)}
          toolsState={toolsState}
        ></GoalManager>
        <ToolsMenu
          requireConf={this._requireConfirmation.bind(this)}
          selectTool={this._selectTool.bind(this)}
          toolsState={toolsState}
        ></ToolsMenu>
        <GraphMap
          className={classes.graphMap}
          // Socket IO
          socket={socket}
          socketConnected={socketConnected}
          // Select path for repeat
          addingGoalPath={addingGoalPath}
          addingGoalType={addingGoalType}
          selectedGoalPath={selectedGoalPath}
          mergePath={mergePath}
          setAddingGoalPath={this._setAddingGoalPath.bind(this)}
          setMergePath={this._setMergePath.bind(this)}
          // Several graph tools from the tools menu
          merge={toolsState.merge}
          relocalize={toolsState.relocalize}
          moveRobot={toolsState.moveRobot}
          moveMap={toolsState.moveMap}
          pinGraph={toolsState.pinGraph}
          addressConf={this._addressConfirmation.bind(this)}
          userConfirmed={userConfirmed}
          // Graph pins
          graphPins={graphPins}
          graphPinType={graphPinType}
          graphPinLatLng={graphPinLatLng}
          graphPinVertex={graphPinVertex}
          setGraphPins={this._setGraphPins.bind(this)}
          setGraphPinVertex={this._setGraphPinVertex.bind(this)}
          setGraphPinLatLng={this._setGraphPinLatLng.bind(this)}
        />
        <GraphPins
          // Socket IO
          socket={socket}
          //
          addressConf={this._addressConfirmation.bind(this)}
          userConfirmed={userConfirmed}
          pinGraph={toolsState.pinGraph}
          graphPins={graphPins}
          graphPinType={graphPinType}
          graphPinLatLng={graphPinLatLng}
          graphPinVertex={graphPinVertex}
          addGraphPin={this._addGraphPin.bind(this)}
          removeGraphPin={this._removeGraphPin.bind(this)}
          setGraphPins={this._setGraphPins.bind(this)}
          setGraphPinType={this._setGraphPinType.bind(this)}
          setGraphPinVertex={this._setGraphPinVertex.bind(this)}
          setGraphPinLatLng={this._setGraphPinLatLng.bind(this)}
          resetGraphPin={this._resetGraphPin.bind(this)}
        />
      </div>
    );
  }

  /**
   * @brief Sets the socketConnected state variable based on true Socket IO
   * connection status.
   * @param {boolean} connected Whether or not Socket IO is connected.
   */
  _handleSocketConnect(connected) {
    console.log("Socket IO connected:", connected);
    this.setState({ socketConnected: connected });
  }

  /**
   * @brief Selects the corresponding tool based on user inputs.
   * @param {string} tool The tool that user selects.
   */
  _selectTool(tool) {
    this.setState((state) => {
      if (tool === null) {
        console.debug("[index] _selectTool: Un-select all tools.");
        Object.keys(state.toolsState).forEach(
          (v) => (state.toolsState[v] = false)
        );
        return {
          toolsState: state.toolsState,
          currTool: null,
        };
      }
      // User selects a tool
      if (!state.currTool) {
        console.debug("[index] _selectTool: User selects", tool);
        return {
          toolsState: {
            ...state.toolsState,
            [tool]: true,
          },
          currTool: tool,
        };
      }
      // User de-selects a tool
      if (state.currTool === tool) {
        console.debug("[index] _selectTool: User de-selects", tool);
        return {
          toolsState: {
            ...state.toolsState,
            [tool]: false,
          },
          currTool: null,
        };
      }
      // User selects a different tool without de-selecting the previous one, so
      // quit the previous one.
      console.debug("[index] _selectTool: User switches to", tool);
      return {
        toolsState: {
          ...state.toolsState,
          [state.currTool]: false,
          [tool]: true,
        },
        currTool: tool,
      };
    });
  }

  /**
   * @brief De-selects the current selected tool and set userConfirmed to true
   * to notify the corresponding handler that user requires a confirmation.
   */
  _requireConfirmation() {
    console.debug("[index] _requireConfirmation");
    this.setState((state) => {
      Object.keys(state.toolsState).forEach(function (key) {
        state.toolsState[key] = false;
      });
      if (state.userConfirmed)
        console.error("Pending user confirmation request!");
      return {
        toolsState: state.toolsState,
        currTool: null,
        userConfirmed: true,
      };
    });
  }

  /**
   * @brief Sets userConfirmed to false to indicate that user confirmation has
   * been addressed.
   *
   * This function must be called every time after calling of
   * _requireConfirmation.
   */
  _addressConfirmation() {
    console.debug("[index] _addressConfirmation");
    this.setState({ userConfirmed: false });
  }

  /**
   * @brief Sets the type of the current goal being added.
   * @param {string} type Type of the goal being added.
   */
  _setAddingGoalType(type) {
    this.setState({ addingGoalType: type });
  }

  /**
   * @brief Sets the target vertices of the current goal being added.
   * For repeat only.
   * @param {array} path Array of vertex ids indicating the repeat path.
   */
  _setAddingGoalPath(path) {
    this.setState({ addingGoalPath: path });
  }

  /**
   * @brief Sets the target vertices of the current selected goal (that is already
   * added).
   * @param {array} path Array of vertex ids indicating the repeat path.
   */
  _setSelectedGoalPath(path) {
    this.setState({ selectedGoalPath: path });
  }

  /**
   * @brief Sets the target vertices for merging.
   * @param {array} path Array of vertex ids indicating the repeat path.
   */
  _setMergePath(path) {
    this.setState({ mergePath: path });
  }

  /** @brief Add a new graph pin to the array of pins. */
  _addGraphPin(newPin) {
    this.setState((state) => {
      return {
        graphPins: [...state.graphPins, newPin],
        // automatically resets everything after adding a pin
        graphPinType: null,
        graphPinVertex: null,
        graphPinLatLng: null,
      };
    });
  }

  /** @brief Remove the pin at index from the array of pins. */
  _removeGraphPin(index) {
    this.setState((state) => {
      state.graphPins.splice(index, 1);
      return { graphPins: state.graphPins };
    });
  }

  /** @brief Sets the current type of pin user want to set, vertex or latlng. */
  _setGraphPins(pins) {
    this.setState({ graphPins: pins });
  }

  /** @brief Sets the current type of pin user want to set, vertex or latlng. */
  _setGraphPinType(pinType) {
    this.setState({ graphPinType: pinType });
  }

  /** @brief Sets the current latlng pin. */
  _setGraphPinLatLng(latLng) {
    this.setState({ graphPinLatLng: latLng });
  }

  /** @brief Sets the current vertex pin (vertex id). */
  _setGraphPinVertex(vertex) {
    this.setState({ graphPinVertex: vertex });
  }

  _resetGraphPin() {
    this.setState({
      graphPinType: null,
      graphPinLatLng: null,
      graphPinVertex: null,
    });
  }
}

var VTRUIStyled = withStyles(styles)(VTRUI);

// ========================================

ReactDOM.render(<VTRUIStyled />, document.getElementById("root"));
