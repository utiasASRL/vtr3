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

import { Box, Button, Tooltip } from "@mui/material";
import DeleteIcon from "@mui/icons-material/Delete";
import DeleteForeverIcon from "@mui/icons-material/DeleteForever";
import CloseIcon from "@mui/icons-material/Close";

class DeleteWaypoints extends React.Component {
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
            <Tooltip title="Delete All Waypoints" placement="top">
              <Button
                sx={{ m: 0.25 }}
                color={"secondary"}
                disableElevation={true}
                variant={"contained"}
                size={"small"}
                onClick={this.handleDeleteAll.bind(this)}
              >
                <DeleteForeverIcon />
              </Button>
            </Tooltip>
            <Tooltip title="Delete Unnamed Waypoints" placement="top">
              <Button
                sx={{ m: 0.25 }}
                color={"primary"}
                disableElevation={true}
                variant={"contained"}
                size={"small"}
                onClick={this.handleDeleteUnnamed.bind(this)}
              >
                <DeleteIcon />
              </Button>
            </Tooltip>
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
            startIcon={<DeleteIcon />}
            variant={"contained"}
            onClick={onSelect}
            size={"small"}
          >
            Delete Waypoints
          </Button>
        )}
      </>
    );
  }

  handleDeleteAll() {
    Array.from(this.props.waypointsMap.keys()).forEach((key) => {
        this.props.handleUpdateWaypoint(key, 1);
    })
  }

  handleDeleteUnnamed() {
    Array.from(this.props.waypointsMap.keys()).forEach((key) => {
        if (this.props.waypointsMap.get(key).slice(0, 3) === "WP-") {
            this.props.handleUpdateWaypoint(key, 1);
        }
    })
  }
}

export default DeleteWaypoints;
