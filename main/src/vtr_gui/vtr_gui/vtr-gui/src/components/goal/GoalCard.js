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

import { Box, Button, Card, Typography } from "@mui/material";
import ClearIcon from "@mui/icons-material/Clear";

/** @brief Parses repeat waypoints and generate a user readable string. */
function parseGoalWaypoints(waypoints) {
  let s = "";
  waypoints.forEach((v) => {
    let vl = parseInt(v.id % Math.pow(2, 32));
    let vh = parseInt((v.id - vl) / Math.pow(2, 32));
    s += vh.toString() + "-" + vl.toString() + ", ";
  });
  s = s.slice(0, s.length - 2);
  return s;
}

class GoalCard extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      expand: false,
    };
  }

  render() {
    const { type, running, waypoints, pauseBefore, pauseAfter } = this.props;
    const { expand } = this.state;
    return (
      <>
        <Card
          sx={{
            width: "100%",
            display: "flex",
            flexDirection: "column",
            justifyContent: "center",
            backgroundColor: running ? "rgba(255, 255, 255, 1.0)" : "rgba(255, 255, 255, 0.8)",
          }}
          onMouseOver={() => this.setState({ expand: true })}
          onMouseOut={() => this.setState({ expand: false })}
        >
          <Box
            sx={{
              width: "100%",
              display: "flex",
              flexDirection: "row",
            }}
          >
            <Typography sx={{ width: 200, m: 1 }} align="center" variant="h5">
              {type.toUpperCase()}
            </Typography>
            <Button
              sx={{ width: 100, m: 1 }}
              color={"primary"}
              disableElevation={true}
              fullWidth={true}
              size="small"
              startIcon={<ClearIcon />}
              variant={"contained"}
              // onClick={(e) => removeGoal(goal, e)}
            >
              Cancel
            </Button>
          </Box>
          {expand && type === "repeat" && (
            <>
              <Box sx={{ display: "flex", width: "100%" }}>
                <Typography sx={{ display: "flex", width: "50%", mx: 1 }} variant="body1">
                  {"BEFORE: " + pauseBefore.toFixed(1) + "s"}
                </Typography>
                <Typography sx={{ display: "flex", width: "50%", mx: 1 }} variant="body1">
                  {"AFTER: " + pauseAfter.toFixed(1) + "s"}
                </Typography>
              </Box>
              <Box sx={{ display: "flex", width: "100%", mx: 1 }}>
                <Typography variant="body1">{"WAYPOINTS: " + parseGoalWaypoints(waypoints)}</Typography>
              </Box>
            </>
          )}
        </Card>
      </>
    );
  }
}

export default GoalCard;
