import clsx from "clsx";
import React from "react";

import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import Card from "@material-ui/core/Card";
import CardContent from "@material-ui/core/CardContent";
import CheckIcon from "@material-ui/icons/Check";
import FormControl from "@material-ui/core/FormControl";
import InputAdornment from "@material-ui/core/InputAdornment";
import InputLabel from "@material-ui/core/InputLabel";
import MenuItem from "@material-ui/core/MenuItem";
import Select from "@material-ui/core/Select";
import TextField from "@material-ui/core/TextField";
import { withStyles } from "@material-ui/core/styles";

const styles = (theme) => ({});

class GoalForm extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
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
    const { disabled, goalPathStr, pauseAfter, pauseBefore } = this.state;
    return (
      <Card>
        {/* Select goal type */}
        <Box m={1} display={"flex"} justifyContent={"center"}>
          <Box m={0.5} width={200}>
            <FormControl>
              <InputLabel>Type</InputLabel>
              <Select
                value={goalType}
                onChange={(e) => this._selectGoalType(e.target.value)}
              >
                <MenuItem value={"Idle"}>Idle</MenuItem>
                <MenuItem value={"Teach"}>Teach</MenuItem>
                <MenuItem value={"Repeat"}>Repeat</MenuItem>
              </Select>
            </FormControl>
          </Box>
          {/* Submit goal */}
          <Box m={0.5} width={100}>
            <Button
              disabled={disabled}
              disableElevation={true}
              color={"secondary"}
              size="small"
              startIcon={<CheckIcon />}
              variant={"contained"}
              onClick={this._submitGoal.bind(this)}
            >
              Confirm
            </Button>
          </Box>
        </Box>
        {/* Get input before and after time */}
        <Box
          m={1}
          display={"flex"}
          justifyContent={"center"}
          flexDirection={"row"}
        >
          <Box m={0.5} display={"flex"} justifyContent={"center"}>
            <TextField
              disabled={disabled}
              fullWidth={true}
              InputProps={{
                endAdornment: <InputAdornment position="end">s</InputAdornment>,
              }}
              label="Before"
              onChange={this._setPauseBefore.bind(this)}
              value={pauseBefore}
            />
          </Box>
          <Box m={0.5} display={"flex"} justifyContent={"center"}>
            <TextField
              disabled={disabled}
              fullWidth={true}
              InputProps={{
                endAdornment: <InputAdornment position="end">s</InputAdornment>,
              }}
              label="After"
              onChange={this._setPauseAfter.bind(this)}
              value={pauseAfter}
            />
          </Box>
        </Box>
        {/* Get input of target vertices */}
        <Box m={1.5} display={"flex"} justifyContent={"center"}>
          {goalType === "Repeat" && (
            <TextField
              disabled={disabled}
              fullWidth={true}
              label="Path"
              onChange={(e) => this.setState({ goalPathStr: e.target.value })}
              onKeyPress={this._setGoalPath.bind(this)}
              value={goalPathStr}
            />
          )}
        </Box>
      </Card>
    );
  }

  /** Selects goal type and clear other input fields.
   *
   * @param {string} type Goal type.
   */
  _selectGoalType(type) {
    this.setState(
      (state, props) => {
        props.setGoalType(type);
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
