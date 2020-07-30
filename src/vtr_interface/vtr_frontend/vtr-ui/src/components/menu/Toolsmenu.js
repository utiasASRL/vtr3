import React from "react";
import { withStyles } from "@material-ui/core/styles";
import IconButton from "@material-ui/core/IconButton";
import Drawer from "@material-ui/core/Drawer";
import clsx from "clsx";

const styles = (theme) => ({
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

    this.state = {};
  }

  render() {
    const { classes, open, toolsState, selectTool, requireConf } = this.props;
    return (
      <Drawer
        className={clsx(classes.drawer)}
        variant="persistent"
        anchor="right"
        open={open}
        classes={{
          paper: classes.drawerPaper,
        }}
      >
        <IconButton
          className={clsx(classes.button, {
            [classes.buttonActive]: toolsState.pinMap,
          })}
          // color="inherit"
          onClick={() => selectTool("pinMap")}
          // aria-label="add goal"
          // edge="start"
        >
          Pin Map
        </IconButton>
        {Object.values(toolsState).some((i) => i) && (
          <IconButton
            className={clsx(classes.button, {
              [classes.buttonActive]: toolsState.pinMap,
            })}
            // color="inherit"
            onClick={() => requireConf()}
            // aria-label="add goal"
            // edge="start"
          >
            Confirm
          </IconButton>
        )}
      </Drawer>
    );
  }
}

export default withStyles(styles)(ToolsMenu);
