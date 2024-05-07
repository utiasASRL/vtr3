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

import { Box, Slider, Paper } from "@mui/material";

const marks = [
  { value: 0, label: "0.1 m" },
  { value: 1, label: "0.5 m" },
  { value: 2, label: "1.0 m" },
  { value: 3, label: "1.5 m" },
  { value: 4, label: "2.0 m" },
  { value: 5, label: "2.5 m" },
  { value: 6, label: "10.0 m" },
];

function valueLabelFormat(value) {
  return marks[marks.findIndex((mark) => mark.value === value)].label;
}

class AnnotateSlider extends React.Component {
  constructor(props) {
    super(props);
    this.state = {};
  }

  render() {
    const { onSliderChange, onSliderChangeCommitted } = this.props;
    return (
      <>
        <Paper
          sx={{
            display: "flex",
            position: "fixed",
            bottom: 30,
            left: "50%",
            transform: "translate(-50%, -50%)",
            opacity: 0.5,
            background: "linear-gradient(90deg, red, orange, yellow, green)",
            width: 550,
            height: 50,
            zIndex: 1000,
          }}
        ></Paper>
        <Box
          sx={{
            display: "flex",
            position: "fixed",
            bottom: 30,
            left: "50%",
            transform: "translate(-50%, -50%)",
            width: 500,
            zIndex: 1100,
          }}
        >
          <Slider
            sx={{ color: "white" }}
            defaultValue={5}
            // color="secondary"
            valueLabelFormat={valueLabelFormat}
            step={null}
            valueLabelDisplay="auto"
            marks={marks}
            min={0}
            max={marks.length - 1}
            onChange={(e, new_value) => onSliderChange(new_value)}
            onChangeCommitted={(e, new_value) => onSliderChangeCommitted(new_value)}
          />
        </Box>
      </>
    );
  }
}

export default AnnotateSlider;
