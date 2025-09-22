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
import PlayCircleIcon from "@mui/icons-material/PlayCircle";

import GoalCard from "./GoalCard";
import GoalCurrent from "./GoalCurrent";
import GoalForm from "./GoalForm";
import AnnotateSlider from "../tools/AnnotateSlider";

//
const GOAL_PANEL_WIDTH = 300;

class MultiRobotGoalManager extends React.Component {
  constructor(props) {
    super(props);
    this.state = { goal_panel_open: false };
  }

  // Helper: get current goals for all robots from serverStates
  getCurrentGoalsByRobot() {
    const { serverStates } = this.props;
    let robotGoals = {};
    if (serverStates) {
      Object.entries(serverStates).forEach(([robot_id, state]) => {
        if (state && state.goals && state.current_goal_id !== undefined) {
          const currentGoal = state.goals.find(g => g.id.toString() === state.current_goal_id.toString());
          if (currentGoal) {
            robotGoals[robot_id] = { ...currentGoal, robot_id };
          }
        }
      });
    }
    return robotGoals;
  }
  render() {
    const {
      socket,
      currentTool,
      selectTool,
      deselectTool,
      serverStates,
      waypointsMap,
      goals,
      currGoalIdx,
      newGoalType,
      newGoalWaypoints,
      setNewGoalType,
      setNewGoalWaypoints,
      followingRouteIds,
      mergeIds,
    } = this.props;
    const { goal_panel_open } = this.state;

    let stateBox;
    // Determine the overall server state across all robots
    if (serverStates) {
      if (Object.values(serverStates).some(state => state === "CRASHED")) {
        stateBox = (<Button
          color={"warning"}
          disableElevation={true}
          variant={"contained"}
          fullWidth={true}
          size={"large"}
        >
          SYSTEM CRASHED
        </Button>);

      } else if (Object.values(serverStates).all(state => state === "PAUSED" || state === "PENDING_PAUSE")) {
        stateBox = (<Button
          color={"primary"}
          disableElevation={true}
          fullWidth={true}
          size={"large"}
          variant={"contained"}
          onClick={this.setPause.bind(this, false)}
        >
          SYSTEM PAUSED
        </Button>);
      }
      else {
        stateBox = (<Button
          color={"primary"}
          disableElevation={true}
          variant={"contained"}
          fullWidth={true}
          size={"large"}
          onClick={this.setPause.bind(this, true)}
        >
          SYSTEM RUNNING
        </Button>);
      }
    }

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
        {stateBox}
        </Box>
        {/* current environment info */}
        {currentTool === null && (
          <AnnotateSlider
            onSliderChange={() => {}}
            onSliderChangeCommitted={this.handleAnnotateSliderChangeCommitted.bind(this)}
          />
        )}
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
            GOALS
          </Button>
        </Box>
        {/* Current goal */}
        {currGoalIdx !== -1 && (
          <GoalCurrent
            socket={socket}
            goal={goals[currGoalIdx]}
            cancelGoal={this.cancelGoal.bind(this)}
            currentTool={currentTool}
            selectTool={selectTool}
            deselectTool={deselectTool}
            followingRouteIds={followingRouteIds}
            mergeIds={mergeIds}
          ></GoalCurrent>
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
            {goals.length > 0 ? (<Button
              sx={{ width: GOAL_PANEL_WIDTH, m: 1 }}
              color={"primary"}
              disableElevation={true}
              fullWidth={true}
              size="large"
              startIcon={<PlayCircleIcon />}
              variant={"contained"}
              onClick={this.beginGoals.bind(this)}
            >
              BEGIN GOALS
            </Button>) : null
            }
            {goals.map((goal, idx) => {
              return (
                <GoalCard
                  key={goal.id}
                  running={idx === currGoalIdx}
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
              waypointsMap={waypointsMap}
              goalWaypoints={newGoalWaypoints}
              goalType={newGoalType}
              setNewGoalWaypoints={setNewGoalWaypoints}
              setGoalType={setNewGoalType}
            ></GoalForm>
          </Box>
        </Drawer>
      </>
    );
  }

  /** @brief Shows/hides the goal panel. */
  toggleGoalPanel() {
    this.setState((state) => ({ goal_panel_open: !state.goal_panel_open }));
  }

  setPause(pause) {
    console.debug("Sending pause signal with pause:", pause);
    this.props.socket.emit("command/set_pause", { pause: pause });
  }
  
  cancelGoal(goal) {
    // Cancel the current goal for each robot
    const robotGoals = this.getCurrentGoalsByRobot();
    Object.entries(robotGoals).forEach(([robot_id, g]) => {
      console.debug(`Sending cancel goal signal for robot ${robot_id} with goal:`, g);
      this.props.socket.emit("command/cancel_goal", g);
    });
  }

  beginGoals() {
    console.debug("Sending begin goals signal");
    this.props.socket.emit("command/begin_goals");
  }

  handleAnnotateSliderChangeCommitted(type) {
    console.debug("Sending annotate slider change signal with type:", type);
    this.props.socket.emit("command/change_env_info", { terrain_type: type });
  }

}

export default GoalManager;
