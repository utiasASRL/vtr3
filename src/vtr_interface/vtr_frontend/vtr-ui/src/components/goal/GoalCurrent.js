import React from "react";
import { withStyles } from "@material-ui/core/styles";
import Card from "@material-ui/core/Card";
import CardActions from "@material-ui/core/CardActions";
import CardContent from "@material-ui/core/CardContent";
import Button from "@material-ui/core/Button";
import Typography from "@material-ui/core/Typography";
import clsx from "clsx";

const styles = (theme) => ({
  root: (props) => {
    const { currGoal } = props;
    let r = currGoal.type === "Idle" ? 255 : 0;
    let g = currGoal.type === "Teach" ? 255 : 0;
    let b = currGoal.type === "Repeat" ? 255 : 0;
    return {
      backgroundColor:
        "rgba(" + String(r) + ", " + String(g) + "," + String(b) + ", 0.7)",
    };
  },
});

class GoalCurrent extends React.Component {
  render() {
    const { classes, className } = this.props;
    return (
      <Card className={clsx(classes.root, className)}>
        <CardContent>
          <Typography variant="h5" component="h2">
            {this.props.currGoal.type +
              this.props.currGoal.id +
              (this.props.currGoalState ? " Running " : " Waiting")}
          </Typography>
        </CardContent>
        <CardActions>
          <Button size="small" onClick={this._cancelCurrentGoal.bind(this)}>
            Cancel
          </Button>
        </CardActions>
      </Card>
    );
  }

  _cancelCurrentGoal() {
    this.props.setCurrGoal({}, false);
  }
}

export default withStyles(styles)(GoalCurrent);
