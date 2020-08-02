import React from "react";
import { withStyles } from "@material-ui/core/styles";
import Card from "@material-ui/core/Card";
import CardActions from "@material-ui/core/CardActions";
import CardContent from "@material-ui/core/CardContent";
import Button from "@material-ui/core/Button";
import Menu from "@material-ui/core/Menu";
import MenuItem from "@material-ui/core/MenuItem";
import TextField from "@material-ui/core/TextField";
import InputAdornment from "@material-ui/core/InputAdornment";
import Typography from "@material-ui/core/Typography";
import clsx from "clsx";

const styles = (theme) => ({
  goalTypeButtion: {},
  pauseTimeInput: {
    width: 120,
  },
});

class GoalForm extends React.Component {
  constructor(props) {
    super(props);

    this.num_goals = 0;

    this.state = {
      anchorEl: null,
      goalType: "Idle",
      path: [],
      pauseBefore: "",
      pauseAfter: "",
    };
  }

  render() {
    const { classes } = this.props;
    const { anchorEl, goalType, pauseBefore, pauseAfter } = this.state;
    return (
      <Card className={classes.root}>
        <CardContent>
          <Typography variant="h5" component="h2">
            Add a goal
          </Typography>
          {/* Select goal type */}
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
          {/* Get input before and after time */}
          <div>
            <TextField
              label="Before"
              value={pauseBefore}
              // id="outlined-start-adornment"
              className={clsx(classes.pauseTimeInput)}
              InputProps={{
                endAdornment: <InputAdornment position="end">s</InputAdornment>,
              }}
              onChange={this._setPauseBefore.bind(this)}
              variant="outlined"
            />
            <TextField
              label="After"
              value={pauseAfter}
              // id="outlined-start-adornment"
              className={clsx(classes.pauseTimeInput)}
              InputProps={{
                endAdornment: <InputAdornment position="end">s</InputAdornment>,
              }}
              onChange={this._setPauseAfter.bind(this)}
              variant="outlined"
            />
          </div>
        </CardContent>
        <CardActions>
          {/* Submit goal */}
          <Button size="small" onClick={this._submitGoal.bind(this)}>
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

  /** Sets pause before and after */
  _setPauseBefore(e) {
    this.setState({ pauseBefore: e.target.value });
  }
  _setPauseAfter(e) {
    console.log("check point");
    this.setState({ pauseAfter: e.target.value });
  }

  _submitGoal() {
    this.setState(
      (state) => {
        console.log(state.goalType);
        console.log(state.pauseBefore);
        this.props.submit({
          type: state.goalType,
          path: state.path,
          pauseBefore: Number(state.pauseBefore),
          pauseAfter: Number(state.pauseAfter),
        });
        console.log("check here");
      },
      () => this._resetGoalForm()
    );
  }

  _resetGoalForm() {
    this.setState({
      goalType: "Idle",
      path: [],
      pauseBefore: "",
      pauseAfter: "",
    });
  }
}

export default withStyles(styles)(GoalForm);
