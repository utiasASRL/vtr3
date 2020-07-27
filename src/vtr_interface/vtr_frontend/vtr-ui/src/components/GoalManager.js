import React from "react";

import { withStyles } from "@material-ui/core/styles";

import Drawer from "@material-ui/core/Drawer";

// Style
const styles = (theme) => ({
  drawer: (props) => ({
    width: props.panel_width,
    flexShrink: 100,
  }),
  drawer_paper: (props) => ({
    width: props.panel_width,
  }),
});

class GoalManager extends React.Component {
  constructor(props) {
    super(props);

    this.state = {};
  }

  componentDidMount() {
    console.log("Goal manager mounted.");
  }

  render() {
    const { classes } = this.props;
    return (
      <>
        <Drawer
          className={classes.drawer}
          variant="persistent"
          anchor="left"
          open={this.props.open}
          classes={{
            paper: classes.drawer_paper,
          }}
        >
          <div>Drawer content.</div>
        </Drawer>
      </>
    );
  }
}

export default withStyles(styles)(GoalManager);
