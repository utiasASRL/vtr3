import React from "react";
import { withStyles } from "@material-ui/core/styles";
import Card from "@material-ui/core/Card";
import CardActions from "@material-ui/core/CardActions";
import CardContent from "@material-ui/core/CardContent";
import Button from "@material-ui/core/Button";
import Typography from "@material-ui/core/Typography";

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

class GoalForm extends React.Component {
  constructor(props) {
    super(props);

    this.idle = 0;
    this.teach = 0;
    this.repeat = 0;
  }

  render() {
    const { classes } = this.props;
    return (
      <Card className={classes.root}>
        <CardContent>
          <Typography variant="h5" component="h2">
            Add a goal
          </Typography>
        </CardContent>
        <CardActions>
          <Button
            size="small"
            onClick={(e) =>
              this.props.submit({ type: "Idle", id: String(this.idle++) })
            }
          >
            Idle
          </Button>
          <Button
            size="small"
            onClick={(e) =>
              this.props.submit({ type: "Teach", id: String(this.teach++) })
            }
          >
            Teach
          </Button>
          <Button
            size="small"
            onClick={(e) =>
              this.props.submit({ type: "Repeat", id: String(this.repeat++) })
            }
          >
            Repeat
          </Button>
        </CardActions>
      </Card>
    );
  }
}

export default withStyles(styles)(GoalForm);
