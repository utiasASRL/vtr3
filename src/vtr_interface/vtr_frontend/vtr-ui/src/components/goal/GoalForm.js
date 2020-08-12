import clsx from "clsx";
import React from "react";

import Button from "@material-ui/core/Button";
import Card from "@material-ui/core/Card";
import CardActions from "@material-ui/core/CardActions";
import CardContent from "@material-ui/core/CardContent";
import InputAdornment from "@material-ui/core/InputAdornment";
import Menu from "@material-ui/core/Menu";
import MenuItem from "@material-ui/core/MenuItem";
import TextField from "@material-ui/core/TextField";
import Typography from "@material-ui/core/Typography";
import { withStyles } from "@material-ui/core/styles";

const styles = (theme) => ({
  goalTypeButtion: {},
  pauseTimeInput: {
    width: 120,
  },
});

class GoalForm extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      anchorEl: null, // Where to pop-up the goalType selection menu.
      disabled: false, // Disable user inputs while waiting for server response.
      goalPathStr: "",
      pauseAfter: "",
      pauseBefore: "",
    };
  }

  componentWillUnmount() {
    // Internal states are reset automatically. External states need to be
    // reset manually here.
    this.props.setGoalType("Idle");
    this.props.setGoalPath([]);
  }

  componentDidUpdate(prevProps) {
    // Update goalPathStr if goalPath is changed by either this or GraphMap.
    if (prevProps.goalPath !== this.props.goalPath)
      this._parseGoalPath(this.props.goalPath);
  }

  render() {
    const { classes, goalType } = this.props;
    const {
      anchorEl,
      disabled,
      goalPathStr,
      pauseAfter,
      pauseBefore,
    } = this.state;
    return (
      <Card className={classes.root}>
        <CardContent>
          <Typography variant="h5">Add a goal</Typography>
          {/* Select goal type */}
          <Button
            className={classes.goalTypeButtion}
            disabled={disabled}
            onClick={(e) => this._setGoalTypeMenu(e.currentTarget)}
            size="small"
            // aria-controls="simple-menu"
            // aria-haspopup="true"
          >
            {goalType}
          </Button>
          <Menu
            anchorEl={anchorEl}
            onClose={() => this._setGoalTypeMenu(null)}
            open={Boolean(anchorEl)}
            // id="simple-menu"
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
                className={clsx(classes.pauseTimeInput)}
                disabled={disabled}
                label="Path"
                onChange={(e) => this.setState({ goalPathStr: e.target.value })}
                onKeyPress={this._setGoalPath.bind(this)}
                value={goalPathStr}
                variant="outlined"
                // id="outlined-start-adornment"
              />
            </div>
          )}
          {/* Get input before and after time */}
          <div>
            <TextField
              className={clsx(classes.pauseTimeInput)}
              disabled={disabled}
              InputProps={{
                endAdornment: <InputAdornment position="end">s</InputAdornment>,
              }}
              label="Before"
              onChange={this._setPauseBefore.bind(this)}
              value={pauseBefore}
              variant="outlined"
              // id="outlined-start-adornment"
            />
            <TextField
              className={clsx(classes.pauseTimeInput)}
              disabled={disabled}
              InputProps={{
                endAdornment: <InputAdornment position="end">s</InputAdornment>,
              }}
              label="After"
              onChange={this._setPauseAfter.bind(this)}
              value={pauseAfter}
              variant="outlined"
              // id="outlined-start-adornment"
            />
          </div>
        </CardContent>
        <CardActions>
          {/* Submit goal */}
          <Button
            disabled={disabled}
            onClick={this._submitGoal.bind(this)}
            size="small"
          >
            Submit Goal
          </Button>
        </CardActions>
      </Card>
    );
  }

  /** Shows/hides the goal type menu.
   *
   * @param {Object} target Target object the menu will be attached to. Hides menu if null.
   */
  _setGoalTypeMenu(target) {
    this.setState({ anchorEl: target });
  }

  /** Selects goal type and clear other input fields.
   *
   * @param {string} type Goal type.
   */
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

  /** Sets pause before. */
  _setPauseBefore(e) {
    this.setState({ pauseBefore: e.target.value });
  }

  /** Sets pause after. */
  _setPauseAfter(e) {
    this.setState({ pauseAfter: e.target.value });
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

  /** Parses repeat path and generate a user readable string. */
  _parseGoalPath(goalPath) {
    let s = "";
    goalPath.forEach((v) => (s += v.toString() + ", "));
    s = s.slice(0, s.length - 1);
    this.setState({
      goalPathStr: s,
    });
  }

  /** Calls GoalManager to submit the goal and disables further modification
   * until reset.
   */
  _submitGoal() {
    this.setState((state, props) => {
      props.submit(
        {
          type: props.goalType,
          path: props.goalPath,
          pauseBefore: Number(state.pauseBefore),
          pauseAfter: Number(state.pauseAfter),
        },
        this._reset.bind(this)
      );
      // Disable until getting response from the server.
      return { disabled: true };
    });
  }

  /** Re-enables user inputs, and resets user input fields if resetGoal is true.
   *
   * This function is only called (as a callback) after submitting the goal.
   * @param {boolean} resetGoal Whether or not to resets user input fields.
   */
  _reset(resetGoal = true) {
    this.setState((state, props) => {
      if (resetGoal) {
        props.setGoalType("Idle");
        props.setGoalPath([]);
        return {
          pauseBefore: "",
          pauseAfter: "",
          disabled: false,
        };
      }
      return { disabled: false };
    });
  }
}

export default withStyles(styles)(GoalForm);
