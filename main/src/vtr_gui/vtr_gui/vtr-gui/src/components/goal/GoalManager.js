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

import { Box, Button, Drawer, List } from "@mui/material";
import ArrowBackIosIcon from "@mui/icons-material/ArrowBackIos";
import ArrowForwardIosIcon from "@mui/icons-material/ArrowForwardIos";
import StorageIcon from "@mui/icons-material/Storage";
import PauseIcon from "@mui/icons-material/Pause";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import StopIcon from "@mui/icons-material/Stop";

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
      status: "PAUSED",
      // goals: [], // {id, type, waypoints, pause_before, pause_after, running} /// \todo
      goals: [
        { id: 1, type: "teach", waypoints: [{ id: 0 }, { id: 1 }], pause_before: 0, pause_after: 0, running: true },
        { id: 2, type: "teach", waypoints: [{ id: 0 }, { id: 1 }], pause_before: 0, pause_after: 0, running: false },
        { id: 3, type: "repeat", waypoints: [{ id: 0 }, { id: 1 }], pause_before: 0, pause_after: 0, running: false },
        { id: 4, type: "repeat", waypoints: [{ id: 0 }, { id: 1 }], pause_before: 0, pause_after: 0, running: false },
      ],
      curr_goal_idx: 0,
    };
    //
    // this.fetchServerState(); /// \todo
  }

  componentDidMount() {
    // Socket IO /// \todo
    // this.props.socket.on("goal/new", this._newGoalCb.bind(this));
    // this.props.socket.on("goal/cancelled", this._removeGoalCb.bind(this));
    // this.props.socket.on("goal/error", this._removeGoalCb.bind(this));
    // this.props.socket.on("goal/success", this._removeGoalCb.bind(this));
    // this.props.socket.on("goal/started", this._startedGoalCb.bind(this));
    // this.props.socket.on("status", this._statusCb.bind(this));
  }

  componentWillUnmount() {
    // Socket IO /// \todo
    // this.props.socket.off("goal/new", this._newGoalCb.bind(this));
    // this.props.socket.off("goal/cancelled", this._removeGoalCb.bind(this));
    // this.props.socket.off("goal/error", this._removeGoalCb.bind(this));
    // this.props.socket.off("goal/success", this._removeGoalCb.bind(this));
    // this.props.socket.off("goal/started", this._startedGoalCb.bind(this));
  }

  render() {
    const { newGoalType, newGoalWaypoints, setNewGoalType, setNewGoalWaypoints } = this.props;
    const { goal_panel_open, status, goals, curr_goal_idx } = this.state;
    return (
      <>
        {/* Button to open/close the goal drawer */}
        <Box
          sx={{
            position: "absolute",
            top: 0,
            left: goal_panel_open ? GOAL_PANEL_WIDTH + 10 : 0,
            width: 120,
            zIndex: 1000,
            m: 0.5,
            transition: "left 0.3s ease-out",
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
        {curr_goal_idx !== -1 && <GoalCurrent type={goals[curr_goal_idx].type}></GoalCurrent>}
        {/* The queue of goals and new goal submission form */}
        <Drawer
          anchor="left"
          elevation={0}
          PaperProps={{
            elevation: 0,
            sx: { backgroundColor: "rgba(255, 255, 255, 0.0)" },
          }}
          variant="persistent"
          open={goal_panel_open}
          transitionDuration={300}
        >
          {/* Start, Pause and Clear buttons */}
          <Box
            sx={{
              width: GOAL_PANEL_WIDTH,
              display: "flex",
              flexDirection: "row",
              justifyContent: "center",
              m: 0.5,
            }}
          >
            <Box sx={{ width: 100, m: 0.5 }}>
              {status === "PAUSED" || status === "PENDING_PAUSE" ? (
                <Button
                  color={"primary"}
                  // disabled={lockStatus} /// \todo
                  disableElevation={true}
                  fullWidth={true}
                  startIcon={<PlayArrowIcon />}
                  size={"small"}
                  variant={"contained"}
                  // onClick={this._handlePlay.bind(this)}
                >
                  Play
                </Button>
              ) : (
                <Button
                  color={"secondary"}
                  // disabled={lockStatus}
                  disableElevation={true}
                  variant={"contained"}
                  fullWidth={true}
                  startIcon={<PauseIcon />}
                  size={"small"}
                  // onClick={this._handlePause.bind(this)} /// \todo
                >
                  Pause
                </Button>
              )}
            </Box>
            <Box sx={{ width: 100, m: 0.5 }}>
              <Button
                color={"primary"}
                // disabled={lockStatus}
                disableElevation={true}
                variant={"contained"}
                fullWidth={true}
                startIcon={<StopIcon />}
                size={"small"}
                // onClick={this._handleClear.bind(this)}
              >
                Clear
              </Button>
            </Box>
          </Box>
          {/* List of goals */}
          <List
            sx={{
              width: GOAL_PANEL_WIDTH,
              overflowY: "auto",
              m: 0.5,
            }}
          >
            {goals.map((goal) => {
              return (
                <GoalCard
                  key={goal.id}
                  type={goal.type}
                  running={goal.running}
                  waypoints={goal.waypoints}
                  pauseBefore={goal.pause_before}
                  pauseAfter={goal.pause_after}
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

  loadServerState(data) {
    console.info("Loading the current server state: ", data);
  }

  /** @brief Shows/hides the goal panel. */
  toggleGoalPanel() {
    this.setState((state) => ({ goal_panel_open: !state.goal_panel_open }));
  }
}

export default GoalManager;
