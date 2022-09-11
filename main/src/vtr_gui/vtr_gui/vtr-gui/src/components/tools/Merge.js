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
import MergeIcon from "@mui/icons-material/Merge";

class MoveRobot extends React.Component {
  render() {
    const { active, onSelect, onCancel } = this.props;
    return (
      <>
        {active ? (
          <Box sx={{ width: 100, m: 1, display: "flex", justifyContent: "center" }}>
            <Button
              color={"secondary"}
              disableElevation={true}
              size={"small"}
              variant={"contained"}
              onClick={this.handleConfirm.bind(this)}
            >
              <CheckIcon />
            </Button>
            <Button color={"primary"} size={"small"} disableElevation={true} variant={"contained"} onClick={onCancel}>
              <CloseIcon />
            </Button>
          </Box>
        ) : (
          <Button
            sx={{ width: 100, m: 1 }}
            color={"primary"}
            disableElevation={true}
            size={"small"}
            startIcon={<MergeIcon />}
            variant={"contained"}
            onClick={onSelect}
          >
            Merge
          </Button>
        )}
      </>
    );
  }

  handleConfirm() {
    this.setState(
      (state, props) => {
        console.debug("Confirmed merge ids: ", props.mergeIds);
        props.socket.emit("command/merge", { ids: props.mergeIds });
      },
      () => this.props.onCancel()
    );
  }
}

export default MoveRobot;
