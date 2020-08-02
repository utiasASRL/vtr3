import React from "react";
import { withStyles } from "@material-ui/core/styles";
import Card from "@material-ui/core/Card";
import CardActions from "@material-ui/core/CardActions";
import CardContent from "@material-ui/core/CardContent";
import Button from "@material-ui/core/Button";
import Menu from "@material-ui/core/Menu";
import MenuItem from "@material-ui/core/MenuItem";
import Typography from "@material-ui/core/Typography";

const styles = (theme) => ({
  goalTypeButtion: {},
});

class GoalForm extends React.Component {
  constructor(props) {
    super(props);

    this.num_goals = 0;

    this.state = {
      anchorEl: null,
      goalType: "Idle",
    };
  }

  render() {
    const { classes } = this.props;
    const { anchorEl, goalType } = this.state;
    return (
      <Card className={classes.root}>
        <CardContent>
          <Typography variant="h5" component="h2">
            Add a goal
          </Typography>
        </CardContent>
        <CardActions>
          <Button
            className={classes.goalTypeButtion}
            size="small"
            // aria-controls="simple-menu"
            // aria-haspopup="true"
            onClick={this._openGoalTypeMenu.bind(this)}
          >
            {goalType}
          </Button>
          <Menu
            // id="simple-menu"
            anchorEl={anchorEl}
            open={Boolean(anchorEl)}
            onClose={this._closeGoalTypeMenu.bind(this)}
          >
            <MenuItem onClick={this._selectGoalType.bind(this, "Idle")}>
              Idle
            </MenuItem>
            <MenuItem onClick={this._selectGoalType.bind(this, "Teach")}>
              Teach
            </MenuItem>
            <MenuItem onClick={this._selectGoalType.bind(this, "Repeat")}>
              Repeat
            </MenuItem>
          </Menu>
          <Button
            size="small"
            onClick={this._submitGoal.bind(this, this.num_goals++)}
          >
            Submit Goal
          </Button>
        </CardActions>
      </Card>
    );
  }

  /** Selects type of goal */
  _openGoalTypeMenu(e) {
    this.setState({ anchorEl: e.currentTarget });
  }
  _closeGoalTypeMenu() {
    this.setState({ anchorEl: null });
  }
  _selectGoalType(type) {
    this.setState({ anchorEl: null, goalType: type });
  }

  _submitGoal(id) {
    this.setState((state) => {
      this.props.submit({ type: state.goalType, id: String(id) });
      return { goalType: "Idle" };
    });
  }
}

export default withStyles(styles)(GoalForm);
