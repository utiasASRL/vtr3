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

import { Box, Button } from "@mui/material";
import CheckIcon from "@mui/icons-material/Check";
import CloseIcon from "@mui/icons-material/Close";
import AndroidIcon from "@mui/icons-material/Android";

class MultiRobotMoveRobot extends React.Component {
  constructor(props) {
    super(props);
    this.state = {};
  }

  render() {
    const { active, onSelect, onCancel } = this.props;
    return (
      <>
        {active ? (
          <Box sx={{ display: "flex", justifyContent: "center" }}>
            <Button
              sx={{ m: 0.25 }}
              color={"secondary"}
              disableElevation={true}
              variant={"contained"}
              size={"small"}
              onClick={this.handleConfirm.bind(this)}
            >
              <CheckIcon />
            </Button>
            <Button
              sx={{ m: 0.25 }}
              color={"primary"}
              disableElevation={true}
              variant={"contained"}
              size={"small"}
              onClick={onCancel}
            >
              <CloseIcon />
            </Button>
          </Box>
        ) : (
          <Button
            sx={{ m: 0.25 }}
            color={"primary"}
            disableElevation={true}
            startIcon={<AndroidIcon />}
            variant={"contained"}
            onClick={onSelect}
            size={"small"}
          >
            Move Robot
          </Button>
        )}
      </>
    );
  }

  handleConfirm() {
    console.log("MultiRobotMoveRobot: handleConfirm called");
    const { moveRobotVertex, socket, robotIds } = this.props;
    console.log("Robot IDs:", robotIds);
    console.log("Move Robot Vertex:", moveRobotVertex);
    // Move every robot with a valid vertex
    if (robotIds && moveRobotVertex) {
      robotIds.forEach((id) => {
        if (moveRobotVertex[id] && moveRobotVertex[id].id !== -1) {
          socket.emit(`command/move_robot`, {
            robot_id: id,
            vertex: moveRobotVertex[id].id,
          });
          console.log(`Emitted move_robot command for robot ${id} to vertex ${moveRobotVertex[id].id}`);
        }
      });
    }
    this.props.onCancel();
  };
}

export default MultiRobotMoveRobot;
