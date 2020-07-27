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
  render() {
    const { classes } = this.props;
    return (
      <Card className={classes.root}>
        <CardContent>
          <Typography variant="h5" component="h2">
            Adding Goal Form
          </Typography>
        </CardContent>
        <CardActions>
          <Button size="small" onClick={this.props.submit}>
            Submit Goal
          </Button>
        </CardActions>
      </Card>
    );
  }
}

export default withStyles(styles)(GoalForm);
