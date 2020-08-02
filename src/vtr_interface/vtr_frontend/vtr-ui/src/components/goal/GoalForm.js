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
      pauseBefore: "",
      pauseAfter: "",
      goalPathStr: "",
    };
  }

  componentDidUpdate(prevProps) {
    // Update goalPathStr if goalPath is changed by either this or GraphMap.
    if (prevProps.goalPath !== this.props.goalPath) {
      this._parseGoalPath(this.props.goalPath);
    }
  }

  render() {
    const { classes, goalType } = this.props;
    const { anchorEl, pauseBefore, pauseAfter, goalPathStr } = this.state;
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
          {/* Get input of target vertices */}
          {goalType === "Repeat" && (
            <div>
              <TextField
                label="Path"
                value={goalPathStr}
                // id="outlined-start-adornment"
                className={clsx(classes.pauseTimeInput)}
                variant="outlined"
                onChange={(e) => this.setState({ goalPathStr: e.target.value })}
                onKeyPress={this._setGoalPath.bind(this)}
              />
            </div>
          )}
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
    this.setState(
      (state, props) => {
        props.setGoalType(type);
        return { anchorEl: null };
      },
      () =>
        this.setState((state, props) => {
          props.setGoalPath([]);
          return {
            pauseBefore: "",
            pauseAfter: "",
          };
        })
    );
  }

  /** Sets pause before and after */
  _setPauseBefore(e) {
    this.setState({ pauseBefore: e.target.value });
  }
  _setPauseAfter(e) {
    this.setState({ pauseAfter: e.target.value });
  }

  /** Parses repeat path. */
  _parseGoalPath(goalPath) {
    let s = "";
    goalPath.forEach((v) => (s += v.toString() + ", "));
    s = s.slice(0, s.length - 1);
    this.setState({
      goalPathStr: s,
    });
  }
  /** Selects repeat path. */
  _setGoalPath(e) {
    if (e.key === "Enter") {
      let input = e.target.value;
      let ids_str = input.replace(/ /g, "").split(",");
      let ids = [];
      for (let id of ids_str) {
        if (!isNaN(parseInt(id.trim()))) ids.push(parseInt(id.trim()));
      }
      this.setState((state, props) => props.setGoalPath(ids));
      e.preventDefault();
    }
  }

  _submitGoal() {
    this.setState(
      (state, props) => {
        props.submit({
          type: props.goalType,
          path: props.goalPath,
          pauseBefore: Number(state.pauseBefore),
          pauseAfter: Number(state.pauseAfter),
        });
      },
      () =>
        this.setState((state, props) => {
          props.setGoalType("Idle");
          props.setGoalPath([]);
          return {
            pauseBefore: "",
            pauseAfter: "",
          };
        })
    );
  }
}

export default withStyles(styles)(GoalForm);
