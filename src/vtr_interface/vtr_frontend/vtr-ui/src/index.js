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

// SocketIO port is assumed to be UI port + 1.
// \todo For now it uses VTR2.1 socket server, change to VTR3.
const socket = io(
  window.location.hostname + ":5001" // (Number(window.location.port) + 1)
);

// Style
const defualt_margin = 5;
const goal_panel_width = 300;
const styles = (theme) => ({
  vtr_ui: (props) => ({
    width: "100%",
    height: "100%",
    position: "absolute",
    // backgroundColor: 'red',
    // color: props => props.color,
  }),
  drawer_button: {
    position: "absolute",
    width: 100,
    height: 64,
    backgroundColor: "red",
    "&:hover": {
      backgroundColor: "blue",
    },
    zIndex: 1000, // \todo This is a magic number.
    marginTop: defualt_margin,
    marginLeft: defualt_margin,
    transition: theme.transitions.create(["margin", "width"], {
      duration: theme.transitions.duration.leavingScreen,
    }),
  },
  drawer_button_shift: {
    marginLeft: goal_panel_width + defualt_margin,
    transition: theme.transitions.create(["margin", "width"], {
      duration: theme.transitions.duration.enteringScreen,
    }),
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

    this.state = { disconnected: false, drawer_open: false };
  }

  componentDidMount() {
    console.log("VTRUI mounted.");

    socket.on("connect", this._handleSocketConnect.bind(this));
    socket.on("disconnect", this._handleSocketDisconnect.bind(this));
  }

  render() {
    const { classes } = this.props;
    return (
      <div className={classes.vtr_ui}>
        <IconButton
          className={clsx(classes.drawer_button, {
            [classes.drawer_button_shift]: this.state.drawer_open,
          })}
          color="inherit"
          aria-label="open drawer"
          onClick={this._toggleDrawer.bind(this)}
          edge="start"
        >
          Goal Panel
        </IconButton>
        <GoalManager
          className={classes.drawer}
          panel_width={goal_panel_width}
          open={this.state.drawer_open}
        ></GoalManager>
        <GraphMap className={classes.graph_map} socket={socket} />
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

  /**Drawer callbacks */
  _toggleDrawer() {
    this.setState((state) => ({ drawer_open: !state.drawer_open }));
  }
}

var VTRUIStyled = withStyles(styles)(VTRUI);

// ========================================

ReactDOM.render(<VTRUIStyled />, document.getElementById("root"));
