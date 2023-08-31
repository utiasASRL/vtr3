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

import { Paper, Box, Button, Drawer, easing } from "@mui/material";
import { Table, TableBody, TableContainer, TableCell, TableHead, TableRow } from "@mui/material";
import ArrowBackIosIcon from "@mui/icons-material/ArrowBackIos";
import ArrowForwardIosIcon from "@mui/icons-material/ArrowForwardIos";
import StorageIcon from "@mui/icons-material/Storage";
import {fetchWithTimeout} from "../../index"


//
const TASK_PANEL_WIDTH = 300;

/** @brief Parses repeat waypoints and generate a user readable string. */
function parseVertexId(v) {
  if (v > Number.MAX_SAFE_INTEGER) return "";
  let vl = parseInt(v % Math.pow(2, 32));
  let vh = parseInt((v - vl) / Math.pow(2, 32));
  return "<" + vh.toString() + "-" + vl.toString() + ">";
}

class TaskQueue extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      tasks: new Map(), // user selected weight for the current pin, -1 means infinity since weight can only be positive.
    };

    //
    this.fetchTaskQueueState();
  }

  componentDidMount() {
    // Socket IO
    this.props.socket.on("task_queue/update", this.taskQueueUpdateCallback.bind(this));
  }

  componentWillUnmount() {
    // Socket IO
    this.props.socket.off("task_queue/update", this.taskQueueUpdateCallback.bind(this));
  }

  render() {
    const { tasks, task_panel_open } = this.state;
    return (
      <>
        {/* Button to open/close the task queue drawer */}
        <Box
          sx={{
            position: "absolute",
            top: 0,
            right: task_panel_open ? TASK_PANEL_WIDTH + 10 : 0,
            width: 120,
            zIndex: 1000,
            m: 1,
            transition: "right 0.3s",
          }}
        >
          <Button
            color={task_panel_open ? "secondary" : "primary"}
            disableElevation={true}
            variant={"contained"}
            fullWidth={true}
            startIcon={task_panel_open ? <ArrowForwardIosIcon /> : <ArrowBackIosIcon />}
            endIcon={<StorageIcon />}
            onClick={this.toggleTaskPanel.bind(this)}
          >
            TASKS
          </Button>
        </Box>
        {/* The queue of tasks */}
        <Drawer
          anchor="right"
          elevation={0}
          PaperProps={{
            elevation: 0,
            sx: { backgroundColor: "rgba(255, 255, 255, 0.0)" },
          }}
          SlideProps={{
            easing: { enter: easing.linear, exit: easing.linear },
          }}
          variant="persistent"
          open={task_panel_open}
          transitionDuration={300}
        >
          <TableContainer
            sx={{
              width: TASK_PANEL_WIDTH,
              backgroundColor: "rgba(255, 255, 255, 0.8)",
              m: 0.5,
            }}
            component={Paper}
          >
            <Table aria-label="graph pins table" size="small">
              <TableHead>
                <TableRow>
                  <TableCell align="center" sx={{ width: "70%" }}>
                    TASK NAME
                  </TableCell>
                  <TableCell align="center" sx={{ width: "30%" }}>
                    VERTEX
                  </TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {/* List of pins already added */}
                {[...tasks.keys()].map((id) => {
                  return (
                    <TableRow>
                      <TableCell align="center">{tasks.get(id).name}</TableCell>
                      <TableCell align="center">{parseVertexId(tasks.get(id).vid)}</TableCell>
                    </TableRow>
                  );
                })}
              </TableBody>
            </Table>
          </TableContainer>
        </Drawer>
      </>
    );
  }

  /** @brief Shows/hides the task panel. */
  toggleTaskPanel() {
    this.setState((state) => ({ task_panel_open: !state.task_panel_open }));
  }

  fetchTaskQueueState() {
    console.info("Fetching the current task queue state (full).");
    fetchWithTimeout("/vtr/task_queue")
      .then((response) => {
        if (response.status !== 200) throw new Error("Failed to fetch task queue state: " + response.status);
        response.json().then((data) => {
          console.info("Received the task queue state (full): ", data);
          this.loadTaskQueueState(data);
        });
      })
      .catch((error) => {
        console.error(error);
      });
  }

  loadTaskQueueState(task_queue) {
    let tasks = new Map();
    task_queue.tasks.forEach((task) => tasks.set(task.id, { name: task.name, vid: task.vid }));
    this.setState({ tasks: tasks });
  }

  /** @brief */
  taskQueueUpdateCallback(task_queue_update) {
    console.info("Received task queue state: ", task_queue_update);
    let task = task_queue_update.task;
    if (task_queue_update.type === 0 /* ADD */) {
      this.setState((state) => {
        state.tasks.set(task.id, { name: task.name, vid: task.vid });
        return { tasks: new Map(state.tasks) };
      });
    } else if (task_queue_update.type === 1 /* REMOVE */) {
      this.setState((state) => {
        state.tasks.delete(task.id);
        return { tasks: new Map(state.tasks) };
      });
    }
  }
}

export default TaskQueue;
