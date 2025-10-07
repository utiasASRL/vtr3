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
import AndroidIcon from "@mui/icons-material/Android";

class ForceAddVertex extends React.Component {
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
            startIcon={<AndroidIcon />}
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
    this.setState(
      (state, props) => {
        console.debug("Confirmed force add vertex");
        props.socket.emit("command/force_add_vertex");
      },
      () => this.props.onCancel()
    );
  }
/*
  fetchRobotState() {
    this.state.robotIds.forEach(id => {
      fetchWithTimeout(`/vtr/robot/${id}`)
        .then((response) => {
          if (response.status !== 200) throw new Error(`Failed to fetch robot state for ${id}: ` + response.status);
          response.json().then((data) => {
            this.loadRobotState(data, id);
          });
        })
        .catch((error) => {
          console.error(error);
        });
    });
  }

  /*
  loadRobotState(state, robot_id) {
    this.setState(prevState => ({
      robotStates: { ...prevState.robotStates, [robot_id]: state }
    }));

    // from vertex
    let vf = graph_update.vertex_from;
    vf.valueOf = () => vf.id;
    vf.distanceTo = L.LatLng.prototype.distanceTo;
    // only update if the vertex is not in the map (vertex position does not change)
    if (!this.id2vertex.has(vf.id)) this.kdtree.insert(vf);
    // always update the vertex map, because vertex neighbors may change
    this.id2vertex.set(vf.id, vf);

    // to vertex
    let vt = graph_update.vertex_to;
    vt.valueOf = () => vt.id;
    vt.distanceTo = L.LatLng.prototype.distanceTo;
    // only update if the vertex is not in the map (vertex position does not change)
    if (!this.id2vertex.has(vt.id)) this.kdtree.insert(vt);
    // always update the vertex map, because vertex neighbors may change
    this.id2vertex.set(vt.id, vt);
  }
  */
}

export default ForceAddVertex;
