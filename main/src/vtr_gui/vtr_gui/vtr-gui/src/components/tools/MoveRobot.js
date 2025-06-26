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
import ArrowRight from "@mui/icons-material/ArrowRight";
import CloseIcon from "@mui/icons-material/Close";
import ArrowLeft from "@mui/icons-material/ArrowLeft";
import AndroidIcon from "@mui/icons-material/Android";

class MoveRobot extends React.Component {
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
              onClick={this.handleConfirm.bind(this, false)}
            >
              Forward <ArrowRight />
            </Button>
            <Button
              sx={{ m: 0.25 }}
              color={"secondary"}
              disableElevation={true}
              variant={"contained"}
              size={"small"}
              onClick={this.handleConfirm.bind(this, true)}
            >
              Backward <ArrowLeft />
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

  handleConfirm(direction) {
    this.setState(
      (state, props) => {
        console.debug("Confirmed move robot: ", props.moveRobotVertex);
        if (props.moveRobotVertex.id !== -1)
          props.socket.emit("command/move_robot", { vertex: props.moveRobotVertex.id, reversed: direction});
      },
      () => this.props.onCancel()
    );
  }
}

export default MoveRobot;
