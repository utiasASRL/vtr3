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

import { Button, Card, Typography } from "@mui/material";
import ClearIcon from "@mui/icons-material/Clear";
import RestartAltIcon from "@mui/icons-material/RestartAlt";
import CheckIcon from "@mui/icons-material/Check";

import Merge from "../tools/Merge";

class GoalCurrent extends React.Component {
  render() {
    const { socket, goal, cancelGoal, currentTool, selectTool, deselectTool, followingRouteIds, mergeIds } = this.props;
    return (
      <Card
        sx={{
          display: "flex",
          position: "fixed",
          top: 80,
          left: "50%",
          zIndex: 1000,
          transform: "translate(-50%, -50%)",
          width: 400,
          flexDirection: "row",
          justifyContent: "center",
          backgroundColor: "rgba(255, 255, 255, 0.8)",
        }}
      >
        <Typography sx={{ width: 200, m: 1 }} align="center" variant="h5">
          {goal.type.toUpperCase() + "ING"}
        </Typography>
        {goal.type === "teach" &&
          (followingRouteIds.length === 0 ? (
            <Merge
              socket={socket}
              active={currentTool === "merge" ? true : false}
              onSelect={() => selectTool("merge")}
              onCancel={deselectTool}
              mergeIds={mergeIds}
            />
          ) : (
            <span>
            <Button
              color={"secondary"}
              disableElevation={true}
              size="small"
              startIcon={<RestartAltIcon />}
              variant={"contained"}
              onClick={() => socket.emit("command/continue_teach", {})}
            >
              Continue Teach
            </Button>
            <Button
            color={"secondary"}
            disableElevation={true}
            size="small"
            startIcon={<CheckIcon />}
            variant={"contained"}
            onClick={() => this.props.socket.emit("command/confirm_merge", {})}
            >
            Accept Merge
            </Button>
            </span>
          ))}
        <Button
          sx={{ width: 100, m: 1 }}
          color={"secondary"}
          disableElevation={true}
          fullWidth={true}
          size="small"
          startIcon={<ClearIcon />}
          variant={"contained"}
          onClick={() => cancelGoal(goal)}
        >
          Cancel
        </Button>
      </Card>
    );
  }
}

export default GoalCurrent;
