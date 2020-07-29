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
const min_gap = 5;
const goal_panel_button_height = 50;
const goal_panel_width = 300;
const styles = (theme) => ({
  vtr_ui: (props) => ({
    width: "100%",
    height: "100%",
    position: "absolute",
    // backgroundColor: 'red',
    // color: props => props.color,
  }),
  tools_menu_button: {
    position: "absolute",
    width: 100,
    height: 50,
    backgroundColor: "rgba(255, 255, 255, 0.7)",
    "&:hover": {
      backgroundColor: "rgba(255, 255, 255, 0.7)",
    },
    zIndex: 1000, // \todo This is a magic number.
    top: min_gap,
    right: min_gap,
    transition: theme.transitions.create(["right", "width"], {
      duration: theme.transitions.duration.leavingScreen,
    }),
  },
  tools_menu_button_shift: {
    right: 200 + min_gap,
    transition: theme.transitions.create(["right", "width"], {
      duration: theme.transitions.duration.enteringScreen,
    }),
  },
  goal_panel_button: {
    position: "absolute",
    width: 100,
    height: goal_panel_button_height,
    backgroundColor: "rgba(255, 255, 255, 0.7)",
    "&:hover": {
      backgroundColor: "rgba(255, 255, 255, 0.7)",
    },
    zIndex: 1000, // \todo This is a magic number.
    top: min_gap,
    left: min_gap,
    transition: theme.transitions.create(["left", "width"], {
      duration: theme.transitions.duration.leavingScreen,
    }),
  },
  goal_panel_button_shift: {
    left: goal_panel_width + min_gap,
    transition: theme.transitions.create(["left", "width"], {
      duration: theme.transitions.duration.enteringScreen,
    }),
  },
  goal_panel: {
    width: goal_panel_width,
  },
  goal_current: {
    position: "absolute",
    top: goal_panel_button_height + 2 * min_gap,
    width: goal_panel_width,
    zIndex: 2000,
  },
  graph_map: {
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
      tools_menu_open: false,
      pin_map: false,
      // Goal manager
      goal_panel_open: false,
      current_goal: {},
      current_goal_state: false,
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
      <div className={classes.vtr_ui}>
        <IconButton
          className={clsx(classes.goal_panel_button, {
            [classes.goal_panel_button_shift]: this.state.goal_panel_open,
          })}
          // color="inherit"
          // aria-label="open drawer"
          onClick={this._toggleGoalPanel.bind(this)}
          // edge="start"
        >
          Goal Panel
        </IconButton>
        {Object.keys(this.state.current_goal).length !== 0 && (
          <GoalCurrent
            className={classes.goal_current}
            currGoal={this.state.current_goal}
            currGoalState={this.state.current_goal_state}
            setCurrGoal={this._setCurrentGoal.bind(this)}
            setCurrGoalState={this._setCurrentGoalState.bind(this)}
          ></GoalCurrent>
        )}
        <GoalManager
          className={classes.goal_panel}
          open={this.state.goal_panel_open}
          currGoal={this.state.current_goal}
          currGoalState={this.state.current_goal_state}
          setCurrGoal={this._setCurrentGoal.bind(this)}
          setCurrGoalState={this._setCurrentGoalState.bind(this)}
        ></GoalManager>
        <IconButton
          className={clsx(classes.tools_menu_button, {
            [classes.tools_menu_button_shift]: this.state.tools_menu_open,
          })}
          // color="inherit"
          // aria-label="open drawer"
          onClick={this._toggleToolsMenu.bind(this)}
          // edge="start"
        >
          Tools Menu
        </IconButton>
        <ToolsMenu
          open={this.state.tools_menu_open}
          pinMap={this.state.pin_map}
          togglePinMap={this._togglePinMap.bind(this)}
        ></ToolsMenu>
        <GraphMap
          className={classes.graph_map}
          socket={socket}
          pinMap={this.state.pin_map}
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
    this.setState((state) => ({ tools_menu_open: !state.tools_menu_open }));
  }
  _togglePinMap() {
    this.setState((state) => ({ pin_map: !state.pin_map }));
  }

  /** Goal manager callbacks */
  _toggleGoalPanel() {
    this.setState((state) => ({ goal_panel_open: !state.goal_panel_open }));
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
      if (goal === state.current_goal) {
        if (run === state.current_goal_state) {
          console.log("[index] setCurrentGoal: Same goal and run, do nothing");
          return;
        } else {
          console.log("[index] setCurrentGoal: Same goal, run => ", run);
          return { current_goal_state: run };
        }
      }
      if (Object.keys(goal).length === 0) {
        console.log("[index] setCurrentGoal: Goal is {}, run => false.");
        return { current_goal: goal, current_goal_state: false };
      } else {
        console.log("[index] setCurrentGoal: Goal set, run => true.");
        return { current_goal: goal, current_goal_state: true };
      }
    });
  }

  _setCurrentGoalState(run) {
    this.setState((state) => {
      if (Object.keys(this.state.current_goal).length === 0) {
        console.log("[index] setCurrentGoalState: No goal, do nothing.");
        return;
      }
      if (run === state.current_goal_state) {
        console.log("[index] setCurrentGoalState: Same state, do nothing.");
        return;
      }
      console.log("[index] setCurrentGoalState: State set to ", run);
      return { current_goal_state: run };
    });
  }
}

var VTRUIStyled = withStyles(styles)(VTRUI);

// ========================================

ReactDOM.render(<VTRUIStyled />, document.getElementById("root"));
