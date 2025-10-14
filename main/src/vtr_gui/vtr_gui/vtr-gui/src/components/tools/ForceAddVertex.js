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
import AddIcon from '@mui/icons-material/Add';
import {fetchWithTimeout} from "../../index"

class ForceAddVertex extends React.Component {
  constructor(props) {
    super(props);
    console.debug("Current props:" + props);
    for (let key in props) {
      console.log(`${key}: ${props[key]}`);
    }
    this.state ={
      current_vertex: null,
    };
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
            startIcon={<AddIcon />}
            variant={"contained"}
            onClick={onSelect}
            size={"small"}
          >
            Add Vertex
          </Button>
        )}
      </>
    );
  }

  handleUpdateWaypoint(vertex_id, type, name="") {
    let update = {
      vertex_id: vertex_id,
      type: type,
      name: name
    };
    console.debug("Updating waypoint:", update);
    this.props.socket.emit("command/update_waypoint", update);
  }

  handleConfirm() {
    let vert_name = window.prompt("Enter a waypoint name for the vertex:","POI");
    this.setState(
      (state, props) => {
        console.debug("Confirmed force add vertex");
        props.socket.emit("command/force_add_vertex");
      },
      () => this.props.onCancel()
    );
    if (vert_name !== null && vert_name !== ""){
      this.fetchRobotState(vert_name);
    }
  }

  fetchRobotState(vert_name) {
    console.info("Fetching the current robot state (full).");
    fetchWithTimeout("/vtr/robot")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch robot state: " + response.status);
        response.json().then((data) => {
          let vertex = this.loadRobotState(data);
          console.warn('Updating waypoint:' + vertex.target.id);
          this.props.handleUpdateWaypoint(vertex.target.id, 0, vert_name); /*ADD*/
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  loadRobotState(robot) {
    let vertex = null;
    if (robot.valid !== false) {
      vertex = this.props.getClosestVertex({lat:robot.lat, lng:robot.lng});
    }
    return vertex;
  }
}

export default ForceAddVertex;
