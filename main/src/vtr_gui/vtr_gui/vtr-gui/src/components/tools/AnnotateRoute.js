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

import { Box, Button, Slider, Paper } from "@mui/material";
import GestureIcon from "@mui/icons-material/Gesture";
import CheckIcon from "@mui/icons-material/Check";
import CloseIcon from "@mui/icons-material/Close";

const marks = [
  { value: 0, label: "Type 1" },
  { value: 1, label: "Type 2" },
  { value: 2, label: "Type 3" },
  { value: 3, label: "Type 4" },
  { value: 4, label: "Type 5" },
  { value: 5, label: "Type 6" },
  { value: 6, label: "Type 7" },
];

function valueLabelFormat(value) {
  return marks[marks.findIndex((mark) => mark.value === value)].label;
}

class AnnotateRoute extends React.Component {
  constructor(props) {
    super(props);
    this.state = {};
  }

  render() {
    const { active, onSelect, onCancel, onSliderChange } = this.props;
    return (
      <>
        {active ? (
          <>
            <Paper
              sx={{
                display: "flex",
                position: "fixed",
                top: "90%",
                left: "50%",
                transform: "translate(-50%, -50%)",
                opacity: 0.5,
                background: "linear-gradient(90deg, red, orange, yellow, green, blue, indigo, violet)",
                width: 550,
                height: 50,
              }}
            ></Paper>
            <Box
              sx={{
                display: "flex",
                position: "fixed",
                top: "90%",
                left: "50%",
                transform: "translate(-50%, -50%)",
                width: 500,
              }}
            >
              <Slider
                sx={{ color: "white" }}
                defaultValue={0}
                // color="secondary"
                valueLabelFormat={valueLabelFormat}
                step={null}
                valueLabelDisplay="auto"
                marks={marks}
                min={0}
                max={marks.length - 1}
                onChange={(e, new_value) => onSliderChange(new_value)}
              />
            </Box>
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
          </>
        ) : (
          <Button
            sx={{ m: 0.25 }}
            color={"primary"}
            disableElevation={true}
            startIcon={<GestureIcon />}
            variant={"contained"}
            size={"small"}
            onClick={onSelect}
          >
            Annotate Graph
          </Button>
        )}
      </>
    );
  }

  handleConfirm() {
    this.setState(
      (state, props) => {
        console.debug("Confirmed annotate route type: ", props.annotateRouteType, ", ids: ", props.annotateRouteIds);
        props.socket.emit("command/annotate_route", {
          type: props.annotateRouteType,
          ids: props.annotateRouteIds,
        });
      },
      () => this.props.onCancel()
    );
  }
}

export default AnnotateRoute;
