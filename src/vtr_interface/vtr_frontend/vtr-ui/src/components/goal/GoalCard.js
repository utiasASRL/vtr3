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
  root: {
    minWidth: 275,
  },
  bullet: {
    display: "inline-block",
    margin: "0 2px",
    transform: "scale(0.8)",
  },
  title: {
    fontSize: 14,
  },
  pos: {
    marginBottom: 12,
  },
});

class GoalCard extends React.Component {
  render() {
    const { classes } = this.props;
    return (
      <Card className={classes.root}>
        <CardContent>
          <Typography variant="h5" component="h2">
            Idle/Teach/Repeat Goal
          </Typography>
          <Typography variant="h5" component="h5">
            {this.props.value.type}
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
