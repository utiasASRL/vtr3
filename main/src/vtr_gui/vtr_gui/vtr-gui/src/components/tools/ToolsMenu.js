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
import MoveGraph from "./MoveGraph";
import MoveRobot from "./MoveRobot";

class ToolsMenu extends React.Component {
  render() {
    const { socket, currentTool, selectTool, deselectTool } = this.props;
    return (
      <Box
        sx={{
          position: "absolute",
          top: 0,
          right: 0,
          width: 170,
          zIndex: 1000,
          display: "flex",
          flexDirection: "column",
        }}
      >
        <MoveRobot
          socket={socket}
          active={currentTool === "move_robot" ? true : false}
          onSelect={() => selectTool("move_robot")}
          onCancel={deselectTool}
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
      </Box>
    );
  }
}

export default ToolsMenu;
