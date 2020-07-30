import React from "react";
import ReactDOM from "react-dom";
import clsx from "clsx"; // define multiple classes conditionally
import io from "socket.io-client";
import "fontsource-roboto"; // for Material UI library, the font package

import { withStyles } from "@material-ui/core/styles";
import IconButton from "@material-ui/core/IconButton";

import "./index.css";

import GraphMap from "./components/graph/GraphMap";
import GoalManager from "./components/goal/GoalManager";
import GoalCurrent from "./components/goal/GoalCurrent";
import ToolsMenu from "./components/menu/Toolsmenu";

// SocketIO port is assumed to be UI port + 1.
// \todo For now it uses VTR2.1 socket server, change to VTR3.
const socket = io(
  window.location.hostname + ":5001" // (Number(window.location.port) + 1)
);

// Style
const minGap = 5;
const goalPanelButtonHeight = 50;
const goalPanelWidth = 300;
const styles = (theme) => ({
  vtrUI: (props) => ({
    width: "100%",
    height: "100%",
    position: "absolute",
    // backgroundColor: 'red',
    // color: props => props.color,
  }),
  toolsMenuButton: {
    position: "absolute",
    width: 100,
    height: 50,
    backgroundColor: "rgba(255, 255, 255, 0.7)",
    "&:hover": {
      backgroundColor: "rgba(255, 255, 255, 0.7)",
    },
    zIndex: 1000, // \todo This is a magic number.
    top: minGap,
    right: minGap,
    transition: theme.transitions.create(["right", "width"], {
      duration: theme.transitions.duration.leavingScreen,
    }),
  },
  toolsMenuButtonShift: {
    right: 200 + minGap,
    transition: theme.transitions.create(["right", "width"], {
      duration: theme.transitions.duration.enteringScreen,
    }),
  },
  goalPanelButton: {
    position: "absolute",
    width: 100,
    height: goalPanelButtonHeight,
    backgroundColor: "rgba(255, 255, 255, 0.7)",
    "&:hover": {
      backgroundColor: "rgba(255, 255, 255, 0.7)",
    },
    zIndex: 1000, // \todo This is a magic number.
    top: minGap,
    left: minGap,
    transition: theme.transitions.create(["left", "width"], {
      duration: theme.transitions.duration.leavingScreen,
    }),
  },
  goalPanelButtonShift: {
    left: goalPanelWidth + minGap,
    transition: theme.transitions.create(["left", "width"], {
      duration: theme.transitions.duration.enteringScreen,
    }),
  },
  goalPanel: {
    width: goalPanelWidth,
  },
  goalCurrent: {
    position: "absolute",
    top: goalPanelButtonHeight + 2 * minGap,
    width: goalPanelWidth,
    zIndex: 2000,
  },
  graphMap: {
    position: "absolute",
    width: "100%",
    height: "100%",
    zIndex: 0,
  },
});

class VTRUI extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      // Socket IO
      disconnected: false,
      // Tools menu
      toolsMenuOpen: false,
      toolsState: { pinMap: false },
      currTool: null,
      userConfirmed: false,
      // Goal manager
      goalPanelOpen: false,
      currentGoal: {},
      currentGoalState: false,
    };
  }

  componentDidMount() {
    console.debug("[index] componentDidMount: VTRUI mounted.");

    socket.on("connect", this._handleSocketConnect.bind(this));
    socket.on("disconnect", this._handleSocketDisconnect.bind(this));
  }

  render() {
    const { classes } = this.props;
    return (
      <div className={classes.vtrUI}>
        <IconButton
          className={clsx(classes.goalPanelButton, {
            [classes.goalPanelButtonShift]: this.state.goalPanelOpen,
          })}
          // color="inherit"
          // aria-label="open drawer"
          onClick={this._toggleGoalPanel.bind(this)}
          // edge="start"
        >
          Goal Panel
        </IconButton>
        {Object.keys(this.state.currentGoal).length !== 0 && (
          <GoalCurrent
            className={classes.goalCurrent}
            currGoal={this.state.currentGoal}
            currGoalState={this.state.currentGoalState}
            setCurrGoal={this._setCurrentGoal.bind(this)}
            setCurrGoalState={this._setCurrentGoalState.bind(this)}
          ></GoalCurrent>
        )}
        <GoalManager
          className={classes.goalPanel}
          open={this.state.goalPanelOpen}
          currGoal={this.state.currentGoal}
          currGoalState={this.state.currentGoalState}
          setCurrGoal={this._setCurrentGoal.bind(this)}
          setCurrGoalState={this._setCurrentGoalState.bind(this)}
        ></GoalManager>
        <IconButton
          className={clsx(classes.toolsMenuButton, {
            [classes.toolsMenuButtonShift]: this.state.toolsMenuOpen,
          })}
          // color="inherit"
          // aria-label="open drawer"
          onClick={this._toggleToolsMenu.bind(this)}
          // edge="start"
        >
          Tools Menu
        </IconButton>
        <ToolsMenu
          open={this.state.toolsMenuOpen}
          toolsState={this.state.toolsState}
          selectTool={this._selectTool.bind(this)}
          requireConf={this._requireConfirmation.bind(this)}
        ></ToolsMenu>
        <GraphMap
          className={classes.graphMap}
          socket={socket}
          pinMap={this.state.toolsState.pinMap}
          userConfirmed={this.state.userConfirmed}
          addressConf={this._addressConfirmation.bind(this)}
        />
      </div>
    );
  }

  /** Socket IO callbacks */
  _handleSocketConnect() {
    this.setState((state, props) => {
      if (state.disconnected === true) return { disconnected: false };
      else return {};
    });
    console.log("Socket IO connected.");
  }
  _handleSocketDisconnect() {
    this.setState({ disconnected: true });
    console.log("Socket IO disconnected.");
  }

  /** Tools menu callbacks */
  _toggleToolsMenu() {
    this.setState((state) => ({ toolsMenuOpen: !state.toolsMenuOpen }));
  }
  _selectTool(tool) {
    this.setState((state) => {
      // User selects a tool
      if (!state.currTool) {
        console.log("[index] _selectTool: User selects ", tool);
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
        console.log("[index] _selectTool: User de-selects ", tool);
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
      console.log("[index] _selectTool: User switches to ", tool);
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
  _requireConfirmation() {
    this.setState((state) => {
      Object.keys(state.toolsState).forEach(function (key) {
        state.toolsState[key] = false;
      });
      return {
        toolsState: state.toolsState,
        currTool: null,
        userConfirmed: true,
      };
    });
  }
  _addressConfirmation() {
    console.log("[index] _addressConfirmation: Confirmation addressed.");
    this.setState({ userConfirmed: false });
  }

  /** Goal manager callbacks */
  _toggleGoalPanel() {
    this.setState((state) => ({ goalPanelOpen: !state.goalPanelOpen }));
  }

  /** Current goal callbacks. Sets the current goal and it's state.
   *
   * Note: If calling _setCurrentGoalState right after this function, it's
   * likely that the setState call is both functions are combined, which may
   * cause unexpected behaviors, so avoid it. Use the default state set in this
   * function, or change it if needed.
   *
   * @param goal The goal to be set to, {} means no goal.
   */
  _setCurrentGoal(goal, run) {
    this.setState((state) => {
      if (goal === state.currentGoal) {
        if (run === state.currentGoalState) {
          console.log("[index] setCurrentGoal: Same goal and run, do nothing");
          return;
        } else {
          console.log("[index] setCurrentGoal: Same goal, run => ", run);
          return { currentGoalState: run };
        }
      }
      if (Object.keys(goal).length === 0) {
        console.log("[index] setCurrentGoal: Goal is {}, run => false.");
        return { currentGoal: goal, currentGoalState: false };
      } else {
        console.log("[index] setCurrentGoal: Goal set, run => true.");
        return { currentGoal: goal, currentGoalState: true };
      }
    });
  }

  _setCurrentGoalState(run) {
    this.setState((state) => {
      if (Object.keys(this.state.currentGoal).length === 0) {
        console.log("[index] setCurrentGoalState: No goal, do nothing.");
        return;
      }
      if (run === state.currentGoalState) {
        console.log("[index] setCurrentGoalState: Same state, do nothing.");
        return;
      }
      console.log("[index] setCurrentGoalState: State set to ", run);
      return { currentGoalState: run };
    });
  }
}

var VTRUIStyled = withStyles(styles)(VTRUI);

// ========================================

ReactDOM.render(<VTRUIStyled />, document.getElementById("root"));
