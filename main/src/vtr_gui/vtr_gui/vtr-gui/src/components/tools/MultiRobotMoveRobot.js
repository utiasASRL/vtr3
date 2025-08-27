/**
 * Multi-robot Move Robot Tool
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
 */

import React from "react";
import { Box, Button, MenuItem, Select, InputLabel, FormControl } from "@mui/material";
import CheckIcon from "@mui/icons-material/Check";
import CloseIcon from "@mui/icons-material/Close";
import AndroidIcon from "@mui/icons-material/Android";

class MultiRobotMoveRobot extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      selectedRobotId: props.robotIds && props.robotIds.length > 0 ? props.robotIds[0] : null,
    };
  }

  handleRobotChange = (event) => {
    this.setState({ selectedRobotId: event.target.value });
    if (this.props.onRobotSelect) {
      this.props.onRobotSelect(event.target.value);
    }
  };

  handleSelect = () => {
    if (this.props.onSelect && this.state.selectedRobotId) {
      this.props.onSelect(this.state.selectedRobotId);
    }
  };

  handleCancel = () => {
    if (this.props.onCancel && this.state.selectedRobotId) {
      this.props.onCancel(this.state.selectedRobotId);
    }
  };

  handleConfirm = () => {
    const { moveRobotVertex, socket } = this.props;
    const { selectedRobotId } = this.state;
    if (
      moveRobotVertex &&
      moveRobotVertex[selectedRobotId] &&
      moveRobotVertex[selectedRobotId].id !== -1
    ) {
      socket.emit("command/move_robot/${selectedRobotId}", {
        robot_id: selectedRobotId,
        vertex: moveRobotVertex[selectedRobotId].id,
      });
    }
    this.handleCancel();
  };

  render() {
    const { active, robotIds } = this.props;
    const { selectedRobotId } = this.state;
    return (
      <>
        <FormControl sx={{ minWidth: 120, m: 0.5 }} size="small">
          <InputLabel id="robot-select-label">Robot</InputLabel>
          <Select
            labelId="robot-select-label"
            value={selectedRobotId || ""}
            label="Robot"
            onChange={this.handleRobotChange}
          >
            {robotIds.map((id) => (
              <MenuItem key={id} value={id}>
                {id}
              </MenuItem>
            ))}
          </Select>
        </FormControl>
        {active ? (
          <Box sx={{ display: "flex", justifyContent: "center" }}>
            <Button
              sx={{ m: 0.25 }}
              color={"secondary"}
              disableElevation={true}
              variant={"contained"}
              size={"small"}
              onClick={this.handleConfirm}
            >
              <CheckIcon />
            </Button>
            <Button
              sx={{ m: 0.25 }}
              color={"primary"}
              disableElevation={true}
              variant={"contained"}
              size={"small"}
              onClick={this.handleCancel}
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
            onClick={this.handleSelect}
            size={"small"}
            disabled={!selectedRobotId}
          >
            Move Robot
          </Button>
        )}
      </>
    );
  }
}

export default MultiRobotMoveRobot;
