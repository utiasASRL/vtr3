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

import { Button, Card, List, TextField, Tooltip } from "@mui/material";

import SetupBackground from "../../images/setup-background.jpg";
import ASRLLogo from "../../images/asrl-logo.png";
import FolderIcon from '@mui/icons-material/Folder';

class StartMenu extends React.Component {constructor(props) {
    super(props);
    this.state = {
      default_dir: "No default dir found",
      available_subdirs: [],
      graph_data_dir: ""
    };
  }

  componentDidMount() {
    // Socket IO
    this.props.socket.on("notification/default_dir", this.defaultDirCallback.bind(this));
    this.props.socket.on("notification/available_subdirs", this.availableSubdirsCallback.bind(this));
    this.props.socket.on("notification/setup_invalid", this.setupInvalidCallback.bind(this));
    this.props.socket.on("notification/setup_complete", this.setupCompleteCallback.bind(this));

    this.props.socket.emit("command/request_default_dir");
    this.props.socket.emit("command/request_available_subdirs");
  }

  componentWillUnmount() {
    // Socket IO
    this.props.socket.off("notification/default_dir", this.defaultDirCallback.bind(this));
    this.props.socket.off("notification/available_subdirs", this.availableSubdirsCallback.bind(this));
    this.props.socket.off("notification/setup_invalid", this.setupInvalidCallback.bind(this));
    this.props.socket.off("notification/setup_complete", this.setupCompleteCallback.bind(this));
  }

  render() {
    return (
      <>
        <div
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            width: '100%',
            height: '100%',
            backgroundImage: "url(" +SetupBackground +")",
            backgroundSize: "cover",
            backgroundPosition: "center",
            zIndex: -1
          }}
        />
        <div
          style={{
            width: "30%",
            marginTop: "8%",
            marginBottom: "5%",
            position: 'relative',
            left: "50%",
          }}
        >
          <Card
            sx={{
              width: "100%",
              p: 2,
              display: "flex",
              flexDirection: "column",
              justifyContent: "center",
              alignItems: "center",
              backgroundColor: "rgba(255, 255, 255, 0.9)",
            }}
          >
            <img
              src={ASRLLogo}
              style={{
                width: "90%",
                margin: "5% 10% 0% 0%",
              }}
              alt="ASRL"
            />
            <div
              style={{
                width: "90%",
                margin: "5%",
                textAlign: "center",
                fontWeight: "bold",
              }}
            >
              Visual Teach & Repeat 3 Setup
            </div>
            <TextField
              sx={{ width: "90%", m: 1 }}
              label="Graph Data Dir"
              variant="standard"
              size="small"
              onChange={(e) => this.setState({ graph_data_dir: e.target.value })}
              value={this.state.graph_data_dir}
            />
            <Button
              sx={{ width: "90%", m: 1, fontWeight: "bold" }}
              color={"primary"}
              disableElevation={true}
              size="small"
              variant={"contained"}
              onClick={this.confirmSetup.bind(this, this.state.graph_data_dir)}
            >
              LAUNCH VTR3
            </Button>
          </Card>
          <div
            style={{
              width: "100%",
              margin: "5% 0% 5% 2.5%",
              textAlign: "center",
              fontWeight: "bold",
              color: "white",
            }}
          >
            OR
          </div>
          <Card
            sx={{
              width: "100%",
              p: 2,
              display: "flex",
              flexDirection: "column",
              justifyContent: "center",
              alignItems: "center",
              backgroundColor: "rgba(255, 255, 255, 0.9)",
            }}
          >
            <List sx={{width: "90%", display: "flex", flexDirection: "column", justifyContent: "left"}}>
              {this.state.available_subdirs.map((item) =>
                <Tooltip title={item} placement="left">
                  <Button
                    sx={{
                      justifyContent: "flex-start",
                      overflow: 'hidden',
                      whiteSpace: 'nowrap',
                    }}
                    disableElevation={true}
                    startIcon={<FolderIcon/>}
                    onClick={this.setGraphDataDir.bind(this, item)}
                  >
                  {item}
                  </Button>
                </Tooltip>
              )}
            </List>
          </Card>
        </div>
      </>
    );
  }

  setGraphDataDir(dir) {
    this.setState({ graph_data_dir: dir })
  }

  defaultDirCallback(dir) {
    console.debug("Received default directory:", dir);
    this.setState({ default_dir: dir, graph_data_dir: dir });
  }

  availableSubdirsCallback(subdirs) {
    console.debug("Received available subdirectories:", subdirs);
    this.setState({ available_subdirs: subdirs });
  }

  setupInvalidCallback() {
    console.debug("Received setup invalid notif");
    alert("The entered setup parameters are invalid. Please ensure that all paths exist.");
  }

  setupCompleteCallback() {
    console.debug("Received setup complete notif, killing setup server");
    this.props.set_setup_complete(true);
    this.props.socket.emit("command/kill_setup_server");
  }

  confirmSetup(dir) {
    let setup_params = {
      data_dir: dir
    };
    console.debug("Sending VTR setup parameters:", setup_params);
    this.props.socket.emit("command/confirm_setup", setup_params);
  }
}

export default StartMenu;