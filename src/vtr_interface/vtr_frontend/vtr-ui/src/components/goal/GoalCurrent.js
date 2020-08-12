import clsx from "clsx";
import React from "react";

import Button from "@material-ui/core/Button";
import Card from "@material-ui/core/Card";
import CardActions from "@material-ui/core/CardActions";
import CardContent from "@material-ui/core/CardContent";
import Typography from "@material-ui/core/Typography";
import { withStyles } from "@material-ui/core/styles";

const styles = (theme) => ({
  root: (props) => {
    const { goal } = props;
    let r = goal.target === "Idle" ? 255 : 150;
    let g = goal.target === "Teach" ? 255 : 150;
    let b = goal.target === "Repeat" ? 255 : 150;
    return {
      backgroundColor:
        "rgba(" + String(r) + ", " + String(g) + "," + String(b) + ", 0.8)",
    };
  },
});

class GoalCurrent extends React.Component {
  render() {
    const {
      active,
      classes,
      className,
      goal,
      onClick,
      removeGoal,
    } = this.props;
    return (
      <Card className={clsx(classes.root, className)} onClick={onClick}>
        <CardContent>
          <Typography variant="h5">{goal.target}</Typography>
          <Typography variant="body1">{"Path: " + goal.path}</Typography>
          <Typography variant="body1">
            {"Before: " + goal.pauseBefore}
          </Typography>
          <Typography variant="body1">{"After: " + goal.pauseAfter}</Typography>
        </CardContent>
        <CardActions>
          {goal.target === "Teach" && (
            <Button size="small" onClick={(e) => {}}>
              Merge
            </Button>
          )}
          {goal.target === "Repeat" && (
            <Button size="small" onClick={(e) => {}}>
              Force Relocalize
            </Button>
          )}
          <Button size="small" onClick={(e) => removeGoal(goal, e)}>
            Cancel
          </Button>
          {active && <Button size="small">*</Button>}
        </CardActions>
      </Card>
    );
  }
}

export default withStyles(styles)(GoalCurrent);
