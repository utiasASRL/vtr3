import clsx from "clsx";
import React from "react";

import Drawer from "@material-ui/core/Drawer";
import IconButton from "@material-ui/core/IconButton";
import { withStyles } from "@material-ui/core/styles";

const minGap = 5;
const toolsMenuWidth = 200;
const styles = (theme) => ({
  toolsMenuButton: {
    backgroundColor: "rgba(255, 255, 255, 0.7)",
    height: 50,
    position: "absolute",
    right: minGap,
    top: minGap,
    transition: theme.transitions.create(["right"], {
      duration: theme.transitions.duration.leavingScreen,
    }),
    width: 100,
    "&:hover": {
      backgroundColor: "rgba(255, 255, 255, 0.7)",
    },
    zIndex: 1000, // \todo Figure out how to set this number properly.
  },
  toolsMenuButtonShift: {
    right: toolsMenuWidth + minGap,
    transition: theme.transitions.create(["right"], {
      duration: theme.transitions.duration.enteringScreen,
    }),
  },
  toolsMenu: {
    flexShrink: 100, // \todo Check if this property has any real effects?
  },
  toolsMenuPaper: {
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    marginTop: 100,
    maxHeight: 200,
    width: toolsMenuWidth,
  },
  confirm: {
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
          classes={{
            paper: classes.toolsMenuPaper,
          }}
          className={clsx(classes.toolsMenu)}
          anchor="right"
          variant="persistent"
          open={toolsMenuOpen}
        >
          <IconButton
            className={clsx(classes.tool, {
              [classes.toolActive]: toolsState.moveMap,
            })}
            onClick={() => selectTool("moveMap")}
            // aria-label="add goal"
            // color="inherit"
            // edge="start"
          >
            Move Graph
          </IconButton>
          <IconButton
            className={clsx(classes.tool, {
              [classes.toolActive]: toolsState.moveRobot,
            })}
            onClick={() => selectTool("moveRobot")}
            // aria-label="add goal"
            // color="inherit"
            // edge="start"
          >
            Move Robot
          </IconButton>
          {(toolsState.moveMap || toolsState.moveRobot) && (
            <IconButton
              className={clsx(classes.confirm)}
              onClick={() => requireConf()}
              // aria-label="add goal"
              // color="inherit"
              // edge="start"
            >
              Confirm
            </IconButton>
          )}
        </Drawer>
      </>
    );
  }

  /** Shows/hides the tools menu. */
  _toggleToolsMenu() {
    this.setState((state) => ({ toolsMenuOpen: !state.toolsMenuOpen }));
  }
}

export default withStyles(styles)(ToolsMenu);
