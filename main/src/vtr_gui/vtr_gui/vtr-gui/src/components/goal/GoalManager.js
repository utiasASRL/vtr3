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

import { Box, Button, Drawer, List, easing } from "@mui/material";
import ArrowBackIosIcon from "@mui/icons-material/ArrowBackIos";
import ArrowForwardIosIcon from "@mui/icons-material/ArrowForwardIos";
import StorageIcon from "@mui/icons-material/Storage";

import GoalCard from "./GoalCard";
import GoalCurrent from "./GoalCurrent";
import GoalForm from "./GoalForm";

//
const GOAL_PANEL_WIDTH = 300;

class GoalManager extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      goal_panel_open: false,
      goal_form_open: false,
      server_state: "EMPTY",
      goals: [], // {id, type, waypoints, pause_before, pause_after}
      curr_goal_idx: -1,
    };
    //
    this.fetchServerState();
  }

  componentDidMount() {
    // Socket IO
    this.props.socket.on("mission/server_state", this.serverStateCallback.bind(this));
  }

  componentWillUnmount() {
    // Socket IO
    this.props.socket.off("mission/server_state", this.serverStateCallback.bind(this));
  }

  render() {
    const { socket, newGoalType, newGoalWaypoints, setNewGoalType, setNewGoalWaypoints } = this.props;
    const { goal_panel_open, server_state, goals, curr_goal_idx } = this.state;
    return (
      <>
        {/* Start, Pause and Clear buttons */}
        <Box
          sx={{
            display: "flex",
            position: "fixed",
            my: 1,
            top: 20,
            left: "50%",
            transform: "translate(-50%, -50%)",
            width: 200,
            zIndex: 1000,
            justifyContent: "center",
          }}
        >
          {server_state === "PAUSED" || server_state === "PENDING_PAUSE" ? (
            <Button
              color={"primary"}
              disableElevation={true}
              fullWidth={true}
              size={"large"}
              variant={"contained"}
              onClick={this.setPause.bind(this, false)}
            >
              SYSTEM PAUSED
            </Button>
          ) : (
            <Button
              color={"warning"}
              disableElevation={true}
              variant={"contained"}
              fullWidth={true}
              size={"large"}
              onClick={this.setPause.bind(this, true)}
            >
              SYSTEM RUNNING
            </Button>
          )}
        </Box>
        {/* Button to open/close the goal drawer */}
        <Box
          sx={{
            position: "absolute",
            top: 0,
            left: goal_panel_open ? GOAL_PANEL_WIDTH + 10 : 0,
            width: 120,
            zIndex: 1000,
            m: 1,
            transition: "left 0.3s",
          }}
        >
          <Button
            color={goal_panel_open ? "secondary" : "primary"}
            disableElevation={true}
            variant={"contained"}
            fullWidth={true}
            startIcon={<StorageIcon />}
            endIcon={goal_panel_open ? <ArrowBackIosIcon /> : <ArrowForwardIosIcon />}
            onClick={this.toggleGoalPanel.bind(this)}
          >
            Goals
          </Button>
        </Box>
        {/* Current goal */}
        {curr_goal_idx !== -1 && (
          <GoalCurrent goal={goals[curr_goal_idx]} cancelGoal={this.cancelGoal.bind(this)}></GoalCurrent>
        )}
        {/* The queue of goals and new goal submission form */}
        <Drawer
          anchor="left"
          elevation={0}
          PaperProps={{
            elevation: 0,
            sx: { backgroundColor: "rgba(255, 255, 255, 0.0)" },
          }}
          SlideProps={{
            easing: { enter: easing.linear, exit: easing.linear },
          }}
          variant="persistent"
          open={goal_panel_open}
          transitionDuration={300}
        >
          {/* List of goals */}
          <List
            sx={{
              width: GOAL_PANEL_WIDTH,
              overflowY: "auto",
              m: 0.5,
            }}
          >
            {goals.map((goal, idx) => {
              return (
                <GoalCard
                  key={goal.id}
                  running={idx === curr_goal_idx}
                  goal={goal}
                  cancelGoal={this.cancelGoal.bind(this)}
                ></GoalCard>
              );
            })}
          </List>
          {/* Goal addition form */}
          <Box
            sx={{
              width: GOAL_PANEL_WIDTH,
              m: 0.5,
              display: "flex",
              flexDirection: "row",
              justifyContent: "center",
            }}
          >
            <GoalForm
              socket={socket}
              goalWaypoints={newGoalWaypoints}
              goalType={newGoalType}
              setGoalWaypoints={setNewGoalWaypoints}
              setGoalType={setNewGoalType}
            ></GoalForm>
          </Box>
        </Drawer>
      </>
    );
  }

  fetchServerState() {
    console.info("Fetching the current server state (full).");
    fetch("/vtr/server")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch server state: " + response.status);
        response.json().then((data) => {
          console.info("Received the server state (full): ", data);
          this.loadServerState(data);
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  serverStateCallback(data) {
    console.info("Received the server state (full): ", data);
    this.loadServerState(data);
  }

  loadServerState(data) {
    console.info("Loading the current server state: ", data);
    this.setState((state) => {
      let curr_goal_idx = -1;
      for (let i = 0; i < data.goals.length; i++) {
        if (data.goals[i].id.toString() === data.current_goal_id.toString()) {
          curr_goal_idx = i;
          break;
        }
      }
      return { server_state: data.server_state, goals: data.goals, curr_goal_idx: curr_goal_idx };
    });
  }

  /** @brief Shows/hides the goal panel. */
  toggleGoalPanel() {
    this.setState((state) => ({ goal_panel_open: !state.goal_panel_open }));
  }

  setPause(pause) {
    console.info("Sending pause signal with pause:", pause);
    this.props.socket.emit("command/set_pause", { pause: pause });
  }

  cancelGoal(goal) {
    console.info("Sending cancel goal signal with goal:", goal);
    this.props.socket.emit("command/cancel_goal", goal);
  }
}

export default GoalManager;
