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
import MergeIcon from "@mui/icons-material/Merge";

class GoalCurrent extends React.Component {
  render() {
    const { type } = this.props;
    return (
      <Card
        sx={{
          display: "flex",
          position: "fixed",
          top: "10%",
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
          {type.toUpperCase() + "ING"}
        </Typography>
        {type === "teach" ? (
          <Button
            sx={{ width: 100, m: 1 }}
            color={"primary"}
            disableElevation={true}
            fullWidth={true}
            size="small"
            startIcon={<MergeIcon />}
            variant={"contained"}
            // onClick={(e) => removeGoal(goal, e)}
          >
            Merge
          </Button>
        ) : (
          <Box sx={{ width: 100, m: 1 }} />
        )}
        <Button
          sx={{ width: 100, m: 1 }}
          color={"secondary"}
          disableElevation={true}
          fullWidth={true}
          size="small"
          startIcon={<ClearIcon />}
          variant={"contained"}
          // onClick={(e) => removeGoal(goal, e)}
        >
          Cancel
        </Button>
      </Card>
    );
  }
}

export default GoalCurrent;
