import React from "react";
import ReactDOM from "react-dom";
import clsx from "clsx"; // define multiple classes conditionally
import io from "socket.io-client";
import "fontsource-roboto"; // for Material UI library, the font package

import { withStyles } from "@material-ui/core/styles";
import Drawer from "@material-ui/core/Drawer";
import IconButton from "@material-ui/core/IconButton";

import "./index.css";

import GraphMap from "./components/GraphMap";

// SocketIO port is assumed to be UI port + 1.
// \todo For now it uses VTR2.1 socket server, change to VTR3.
const socket = io(
  window.location.hostname + ":" + "5001" // (Number(window.location.port) + 1)
);

// Style
const drawer_width = 256;
const styles = (theme) => ({
  vtr_ui: (props) => ({
    width: "100%",
    height: "100%",
    position: "absolute",
    // backgroundColor: 'red',
    // color: props => props.color,
  }),
  drawer: {
    width: drawer_width,
    flexShrink: 100,
  },
  drawer_button: {
    position: "absolute",
    width: 100,
    height: 64,
    backgroundColor: "red",
    "&:hover": {
      backgroundColor: "blue",
    },
    zIndex: 1000, // \todo This is a magic number.
    marginLeft: 0,
    transition: theme.transitions.create(["margin", "width"], {
      duration: theme.transitions.duration.leavingScreen,
    }),
  },
  drawer_button_shift: {
    marginLeft: drawer_width,
    transition: theme.transitions.create(["margin", "width"], {
      duration: theme.transitions.duration.enteringScreen,
    }),
  },
  drawer_paper: {
    width: 240,
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
          Button
        </IconButton>
        <Drawer
          className={classes.drawer}
          variant="persistent"
          anchor="left"
          open={this.state.drawer_open}
          classes={{
            paper: classes.drawer_paper,
          }}
        >
          <div>Drawer content.</div>
        </Drawer>
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
