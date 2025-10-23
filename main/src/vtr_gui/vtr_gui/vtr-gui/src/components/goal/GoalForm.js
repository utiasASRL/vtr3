/**
 * Copyright 2021, Autonomous Space Robotics Lab (ASRL)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

import React from "react";

import { Box, Button, ButtonGroup, Card, IconButton, InputAdornment, TextField } from "@mui/material";
import CheckIcon from "@mui/icons-material/Check";
import CloseIcon from "@mui/icons-material/Close";
import AddIcon from "@mui/icons-material/Add";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";
import ClearIcon from "@mui/icons-material/Clear";

class GoalForm extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      goal_form_open: false,
      goal_waypoints_str: "",
      goal_waypoints_invalid: false,
      pause_after: "",
      pause_before: "",
    };
  }

  componentDidUpdate(prevProps) {
    // Update goal_waypoints_str if goalWaypoints is changed by either this or GraphMap.
    if (prevProps.goalWaypoints !== this.props.goalWaypoints || prevProps.waypointsMap !== this.props.waypointsMap) 
    this.updateGoalWaypointsStr(this.props.goalWaypoints);
  }

  render() {
    const { goalType, goalWaypoints, setNewGoalWaypoints, cancelAllGoals } = this.props;
    const { goal_form_open, goal_waypoints_str, goal_waypoints_invalid, pause_after, pause_before } = this.state;
    return (
      <>
        {goal_form_open ? (
          <Card
            sx={{
              width: "100%",
              display: "flex",
              flexDirection: "column",
              justifyContent: "center",
              backgroundColor: "rgba(255, 255, 255, 0.6)",
            }}
          >
            {/* Select goal type */}
            <ButtonGroup
              sx={{ width: "100%", mt: 1, display: "flex", flexDirection: "row", justifyContent: "center" }}
              disableElevation={true}
              variant={"contained"}
            >
              <Button
                sx={{ width: 120 }}
                color={goalType === "teach" ? "secondary" : "primary"}
                onClick={this.selectGoalType.bind(this, "teach")}
              >
                Teach
              </Button>
              <Button
                sx={{ width: 120 }}
                color={goalType === "repeat" ? "secondary" : "primary"}
                onClick={this.selectGoalType.bind(this, "repeat")}
              >
                Repeat
              </Button>
              <Button
                sx={{ width: 120 }}
                color={goalType === "localize" ? "secondary" : "primary"}
                onClick={this.selectGoalType.bind(this, "localize")}
              >
                Localize
              </Button>
            </ButtonGroup>
            {goalType === "repeat" && (
              <>
                {/* Get input before and after time */}
                <Box sx={{ mx: 1, my: 0.5, display: "flex", justifyContent: "center", flexDirection: "row" }}>
                  <TextField
                    sx={{ mx: 0.5, display: "flex", justifyContent: "center" }}
                    fullWidth={true}
                    InputProps={{
                      endAdornment: <InputAdornment position="end">s</InputAdornment>,
                    }}
                    label="before"
                    variant="standard"
                    size="small"
                    onChange={this.setPauseBefore.bind(this)}
                    value={pause_before}
                  />
                  <TextField
                    sx={{ mx: 0.5, display: "flex", justifyContent: "center" }}
                    fullWidth={true}
                    InputProps={{
                      endAdornment: <InputAdornment position="end">s</InputAdornment>,
                    }}
                    label="after"
                    variant="standard"
                    size="small"
                    onChange={this.setPauseAfter.bind(this)}
                    value={pause_after}
                  />
                </Box>
                {/* Get input waypoints to repeat */}
                <Box sx={{ mx: 1, my: 0.5, display: "flex", justifyContent: "center", flexDirection: "row" }}>
                  <TextField
                    sx={{ mx: 0.5 }}
                    fullWidth={true}
                    label="waypoints"
                    variant="standard"
                    size="small"
                    error={goal_waypoints_invalid}
                    onChange={(e) => {
                      this.setState({ goal_waypoints_str: e.target.value });
                      this.parseGoalWaypoints(e.target.value);
                    }}
                    value={goal_waypoints_str}
                  />
                  <IconButton
                    color="secondary"
                    size="small"
                    onClick={() => {
                      if (goalWaypoints.length > 0) {
                        let new_goal_waypoints = goalWaypoints.slice(0, goalWaypoints.length - 1);
                        setNewGoalWaypoints(new_goal_waypoints);
                      }
                    }}
                  >
                    <ArrowBackIcon />
                  </IconButton>
                  <IconButton color="secondary" size="small" onClick={() => setNewGoalWaypoints([])}>
                    <ClearIcon />
                  </IconButton>
                </Box>
              </>
            )}
            {/* Confirm or cancel */}
            <Box sx={{ width: "100%", display: "flex", my: 0.5, flexDirection: "row", justifyContent: "center" }}>
              <Button
                sx={{ width: 100, mx: 1 }}
                disabled={goal_waypoints_invalid || goalType === ""}
                color={"secondary"}
                disableElevation={true}
                size="small"
                startIcon={<CheckIcon />}
                variant={"contained"}
                onClick={this.addGoal.bind(this)}
              >
                Confirm
              </Button>
              <Button
                sx={{ width: 100, mx: 1 }}
                color={"secondary"}
                disableElevation={true}
                size="small"
                startIcon={<CloseIcon />}
                variant={"contained"}
                onClick={this.toggleGoalForm.bind(this, false)}
              >
                Cancel
              </Button>
            </Box>
          </Card>
        ) : (
          <Box sx={{ width: "100%", display: "flex", flexDirection: "row", justifyContent: "center" }}>
            <Button
              color={"primary"}
              disableElevation={true}
              variant={"contained"}
              startIcon={<AddIcon />}
              size={"small"}
              onClick={this.toggleGoalForm.bind(this, true)}
            >
              Add Goal
            </Button>
            <Button
              color={"primary"}
              disableElevation={true}
              variant={"contained"}
              startIcon={<ClearIcon />}
              size={"small"}
              onClick={cancelAllGoals.bind(this)}
            >
              Clear All Goals
            </Button>
          </Box>
        )}
      </>
    );
  }

  /** @brief Shows/hides the goal addition form. */
  toggleGoalForm(enable) {
    // clear existing goal
    this.props.setGoalType("");
    this.props.setNewGoalWaypoints([]);
    this.setState({
      goal_form_open: enable ? true : false,
      goal_waypoints_invalid: false,
      goal_waypoints_str: "",
      pause_after: "",
      pause_before: "",
    });
  }

  /**
   * @brief Selects goal type and clear other input fields.
   * @param {string} type Goal type.
   */
  selectGoalType(type) {
    this.props.setGoalType(type);
    this.props.setNewGoalWaypoints([]);
    this.setState({ goal_waypoints_invalid: false, goal_waypoints_str: "", pause_after: "", pause_before: "" });
  }

  /** @brief Sets pause before. */
  setPauseBefore(e) {
    console.info("Setting pause before to ", e.target.value);
    this.setState({ pause_before: e.target.value });
  }

  /** @brief Sets pause after. */
  setPauseAfter(e) {
    console.info("Setting pause after to ", e.target.value);
    this.setState({ pause_after: e.target.value });
  }

  /** @brief Selects repeat waypoints. */
  parseGoalWaypoints(input) {
    let names_str = input.replace(/ /g, "").split(",");
    let ids = [];
    for (let name of names_str) {
      if (Array.from(this.props.waypointsMap.values()).includes(name)){
        ids.push(
          Array.from(this.props.waypointsMap.keys()).find(
            (key) => this.props.waypointsMap.get(key) === name
          )
        );
      }
      else {
        this.setState({ goal_waypoints_invalid: true });
        return;
      }
    }
    this.props.setNewGoalWaypoints(ids);
    this.setState({ goal_waypoints_invalid: false });
  }

  /** @brief Parses repeat waypoints and generate a user readable string. */
  updateGoalWaypointsStr(goal_waypoints) {
    let s = "";
    goal_waypoints.forEach((id) => {
      if (this.props.waypointsMap.has(id)) s += this.props.waypointsMap.get(id) + ", ";
    });
    s = s.slice(0, s.length - 2);
    this.setState({
      goal_waypoints_str: s,
      goal_waypoints_invalid: false,
    });
  }

  /** @brief Calls GoalManager to submit the goal */
  addGoal() {
    this.setState(
      (state, props) => {
        let goal = {
          type: props.goalType,
          pause_before: Number(state.pause_before),
          pause_after: Number(state.pause_after),
          waypoints: props.goalWaypoints
        };
        console.debug("Create new goal:", goal);
        props.socket.emit("command/add_goal", goal);
        return { goal_waypoints_invalid: false, goal_waypoints_str: "", pause_after: "", pause_before: "" };
      },
      () => {
        this.props.setGoalType("");
        this.props.setNewGoalWaypoints([]);
      }
    );
  }
}

export default GoalForm;
