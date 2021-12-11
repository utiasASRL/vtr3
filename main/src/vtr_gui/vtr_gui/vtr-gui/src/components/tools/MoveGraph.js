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
import GestureIcon from "@mui/icons-material/Gesture";
import CheckIcon from "@mui/icons-material/Check";
import CloseIcon from "@mui/icons-material/Close";

class MoveGraph extends React.Component {
  constructor(props) {
    super(props);

    this.state = {};
  }

  render() {
    const { active, onSelect, onCancel } = this.props;
    return (
      <>
        {active ? (
          <Box
            sx={{
              display: "flex",
              justifyContent: "center",
            }}
          >
            <Button
              sx={{
                m: 0.25,
              }}
              color={"secondary"}
              disableElevation={true}
              variant={"contained"}
              onClick={this.handleConfirm.bind(this)}
            >
              <CheckIcon />
            </Button>
            <Button
              sx={{
                m: 0.25,
              }}
              color={"primary"}
              disableElevation={true}
              variant={"contained"}
              onClick={onCancel}
            >
              <CloseIcon />
            </Button>
          </Box>
        ) : (
          <Button
            sx={{
              m: 0.25,
            }}
            color={"primary"}
            disableElevation={true}
            startIcon={<GestureIcon />}
            variant={"contained"}
            onClick={onSelect}
          >
            Move Graph
          </Button>
        )}
      </>
    );
  }

  handleConfirm() {
    this.setState(
      (state, props) => {
        console.debug("Confirmed move graph: ", props.moveGraphChange);
        props.socket.emit("command/move_graph", {
          lng: props.moveGraphChange.lng,
          lat: props.moveGraphChange.lat,
          theta: props.moveGraphChange.theta,
          scale: props.moveGraphChange.scale,
        });
      },
      () => this.props.onCancel()
    );
  }
}

export default MoveGraph;
