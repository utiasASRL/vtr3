import React from "react";

import ArrowBackIcon from "@material-ui/icons/ArrowBack";
import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import Card from "@material-ui/core/Card";
import CheckIcon from "@material-ui/icons/Check";
import ClearIcon from "@material-ui/icons/Clear";
import FormControl from "@material-ui/core/FormControl";
import IconButton from "@material-ui/core/IconButton";
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
    const { goalType, setGoalPath } = this.props;
    const { disabled, goalPathStr, pauseAfter, pauseBefore } = this.state;
    return (
      <Card>
        {/* Select goal type */}
        <Box mb={1} width={1} display={"flex"} flexDirection={"row"}>
          <Box width={0.7} my={"auto"} ml={1.5} mt={2.5}>
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
          <Box width={100} m={1}>
            <Button
              color={"secondary"}
              fullWidth={true}
              disabled={disabled}
              disableElevation={true}
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
          mx={1}
          mb={1}
          display={"flex"}
          justifyContent={"center"}
          flexDirection={"row"}
        >
          <Box mx={0.5} display={"flex"} justifyContent={"center"}>
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
          <Box mx={0.5} display={"flex"} justifyContent={"center"}>
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
        {goalType === "Repeat" && (
          <Box mx={1.5} mb={1} display={"flex"} justifyContent={"center"}>
            <TextField
              disabled={disabled}
              fullWidth={true}
              label="Path"
              onChange={(e) => this.setState({ goalPathStr: e.target.value })}
              onKeyPress={this._setGoalPath.bind(this)}
              value={goalPathStr}
            />
            <IconButton
              color="secondary"
              disabled={disabled}
              onClick={() => {
                this.setState((state, props) => {
                  props.setGoalPath(
                    props.goalPath.slice(0, props.goalPath.length - 1)
                  );
                });
              }}
            >
              <ArrowBackIcon />
            </IconButton>
            <IconButton
              color="secondary"
              disabled={disabled}
              onClick={() => setGoalPath([])}
            >
              <ClearIcon />
            </IconButton>
          </Box>
        )}
      </Card>
    );
  }

  /**
   * @brief Selects goal type and clear other input fields.
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

  /** @brief Sets pause before. */
  _setPauseBefore(e) {
    this.setState({ pauseBefore: e.target.value });
  }

  /** @brief Sets pause after. */
  _setPauseAfter(e) {
    this.setState({ pauseAfter: e.target.value });
  }

  /** @brief Selects repeat path. */
  _setGoalPath(e) {
    if (e.key === "Enter") {
      let input = e.target.value;
      let ids_str = input.replace(/ /g, "").split(",");
      let ids = [];
      for (let id of ids_str) {
        let idpair = id.split("-");
        for (let i of idpair) if (isNaN(i)) continue;

        if (idpair.length === 1) ids.push(parseInt(idpair[0]));
        else if (idpair.length === 2)
          ids.push(parseInt(idpair[0]) * Math.pow(2, 32) + parseInt(idpair[1]));
      }
      this.setState((state, props) => props.setGoalPath(ids));
      e.preventDefault();
    }
  }

  /** @brief Parses repeat path and generate a user readable string. */
  _parseGoalPath(goalPath) {
    let s = "";
    goalPath.forEach((v) => {
      let vl = parseInt(v % Math.pow(2, 32));
      let vh = parseInt((v - vl) / Math.pow(2, 32));
      s += vh.toString() + "-" + vl.toString() + ", ";
    });
    s = s.slice(0, s.length - 2);
    this.setState({
      goalPathStr: s,
    });
  }

  /**
   * @brief Calls GoalManager to submit the goal and disables further
   * modification until reset.
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
