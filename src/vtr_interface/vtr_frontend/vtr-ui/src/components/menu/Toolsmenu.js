import React from "react";
import { withStyles } from "@material-ui/core/styles";
import IconButton from "@material-ui/core/IconButton";
import Drawer from "@material-ui/core/Drawer";
import clsx from "clsx";

const minGap = 5;
const styles = (theme) => ({
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
  drawer: (props) => ({
    flexShrink: 100,
    marginRight: 100,
  }),
  drawerPaper: {
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    top: 100,
    maxHeight: 200,
    width: 200,
  },
  button: {
    backgroundColor: "rgba(255, 255, 255, 0.6)",
  },
  buttonActive: {
    backgroundColor: "rgba(255, 255, 0, 0.6)",
  },
});

class ToolsMenu extends React.Component {
  constructor(props) {
    super(props);

    this.state = { toolsMenuOpen: false };
  }

  render() {
    const { classes, toolsState, selectTool, requireConf } = this.props;
    const { toolsMenuOpen } = this.state;
    return (
      <>
        <IconButton
          className={clsx(classes.toolsMenuButton, {
            [classes.toolsMenuButtonShift]: toolsMenuOpen,
          })}
          onClick={this._toggleToolsMenu.bind(this)}
          // color="inherit"
          // aria-label="open drawer"
          // edge="start"
        >
          Tools Menu
        </IconButton>
        <Drawer
          className={clsx(classes.drawer)}
          variant="persistent"
          anchor="right"
          open={toolsMenuOpen}
          classes={{
            paper: classes.drawerPaper,
          }}
        >
          <IconButton
            className={clsx(classes.button, {
              [classes.buttonActive]: toolsState.pinMap,
            })}
            onClick={() => selectTool("pinMap")}
            // color="inherit"
            // aria-label="add goal"
            // edge="start"
          >
            Pin Map
          </IconButton>
          <IconButton
            className={clsx(classes.button, {
              [classes.buttonActive]: false, // \todo Add active condition
            })}
            onClick={() => {}} // \todo add selectTool
            // color="inherit"
            // aria-label="add goal"
            // edge="start"
          >
            Set Localization
          </IconButton>
          {Object.values(toolsState).some((i) => i) && (
            <IconButton
              className={clsx(classes.button, {
                [classes.buttonActive]: toolsState.pinMap,
              })}
              onClick={() => requireConf()}
              // color="inherit"
              // aria-label="add goal"
              // edge="start"
            >
              Confirm
            </IconButton>
          )}
        </Drawer>
      </>
    );
  }

  /** Shows/hides tools menu. */
  _toggleToolsMenu() {
    this.setState((state) => ({ toolsMenuOpen: !state.toolsMenuOpen }));
  }
}

export default withStyles(styles)(ToolsMenu);
