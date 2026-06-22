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

import { Box } from "@mui/material";

import AnnotateRoute from "./AnnotateRoute";
import DeleteWaypoints from "./DeleteWaypoints";
import MoveGraph from "./MoveGraph";
import MultiRobotMoveRobot from "./MultiRobotMoveRobot";
import ForceAddVertex from "./ForceAddVertex";

class MultiRobotToolsMenu extends React.Component {
  render() {
    const { socket, currentTool, selectTool, deselectTool, robotIds } = this.props;
    return (
      <Box
        sx={{
          position: "absolute",
          bottom: 0,
          left: "50%",
          transform: "translate(-50%, -50%)",
          zIndex: 1000,
          display: "flex",
          flexDirection: "row",
        }}
      >
        <MultiRobotMoveRobot
          socket={socket}
          active={currentTool === "move_robot" ? true : false}
          onSelect={() => selectTool("move_robot")}
          onCancel={deselectTool}
          moveRobotVertex={this.props.moveRobotVertex}
          robotIds={this.props.robotIds}
        />
        <MoveGraph
          socket={socket}
          active={currentTool === "move_graph" ? true : false}
          onSelect={() => selectTool("move_graph")}
          onCancel={deselectTool}
          moveGraphChange={this.props.moveGraphChange}
        />
        <AnnotateRoute
          socket={socket}
          active={currentTool === "annotate_route" ? true : false}
          onSelect={() => selectTool("annotate_route")}
          onCancel={deselectTool}
          onSliderChange={this.props.onSliderChange}
          annotateRouteType={this.props.annotateRouteType}
          annotateRouteIds={this.props.annotateRouteIds}
        />
        <DeleteWaypoints
          socket={socket}
          active={currentTool === "delete_waypoints" ? true : false}
          onSelect={() => selectTool("delete_waypoints")}
          onCancel={deselectTool}
          waypointsMap={this.props.waypointsMap}
          handleUpdateWaypoint={this.props.handleUpdateWaypoint}
        />
        <ForceAddVertex
          socket={socket}
          active={currentTool === "force_add_vertex" ? true : false}
          onSelect={() => selectTool("force_add_vertex")}
          onCancel={deselectTool}
          waypointsMap={this.props.waypointsMap}
          handleUpdateWaypoint={this.props.handleUpdateWaypoint}
          getClosestVertex={this.props.getClosestVertex}
        />
      </Box>
    );
  }
}

export default MultiRobotToolsMenu;
