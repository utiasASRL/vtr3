import React from "react";
import { withStyles } from "@material-ui/core/styles";
import Card from "@material-ui/core/Card";
import CardActions from "@material-ui/core/CardActions";
import CardContent from "@material-ui/core/CardContent";
import Button from "@material-ui/core/Button";
import Typography from "@material-ui/core/Typography";

import { sortableHandle } from "react-sortable-hoc";

const DragHandle = sortableHandle(() => <Button size="small">Move</Button>);

const styles = (theme) => ({
  root: (props) => {
    const { goal } = props;
    let r = goal.type === "Idle" ? 255 : 0;
    let g = goal.type === "Teach" ? 255 : 0;
    let b = goal.type === "Repeat" ? 255 : 0;
    return {
      backgroundColor:
        "rgba(" + String(r) + ", " + String(g) + "," + String(b) + ", 0.2)",
    };
  },
});

class GoalCard extends React.Component {
  render() {
    const { classes } = this.props;
    return (
      <Card className={classes.root}>
        <CardContent>
          <Typography variant="h5" component="h2">
            Goal card
          </Typography>
          <Typography variant="h5" component="h5">
            {this.props.goal.type + this.props.goal.id}
          </Typography>
        </CardContent>
        <CardActions>
          <DragHandle></DragHandle>
          <Button
            size="small"
            onClick={(e) => this.props.delete(this.props.id, e)}
          >
            Cancel
          </Button>
        </CardActions>
      </Card>
    );
  }
}

export default withStyles(styles)(GoalCard);
