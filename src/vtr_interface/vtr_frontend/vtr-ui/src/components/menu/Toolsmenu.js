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
  drawer_paper: {
    backgroundColor: "rgba(255, 255, 255, 0.2)",
    top: 100,
    maxHeight: 200,
    width: 200,
  },
});

class ToolsMenu extends React.Component {
  constructor(props) {
    super(props);

    this.state = {};
  }

  render() {
    const { classes } = this.props;
    return (
      <Drawer
        className={clsx(classes.drawer)}
        variant="persistent"
        anchor="right"
        open={this.props.open}
        classes={{
          paper: classes.drawer_paper,
        }}
      >
        <IconButton
          // color="inherit"
          onClick={() => this.props.togglePinMap()}
          // aria-label="add goal"
          // edge="start"
        >
          Pin Map
        </IconButton>
      </Drawer>
    );
  }
}

export default withStyles(styles)(ToolsMenu);
