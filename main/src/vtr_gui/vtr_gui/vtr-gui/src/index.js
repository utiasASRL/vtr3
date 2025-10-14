import React from "react";
import ReactDOM from "react-dom";
import "./index.css";

import io from "socket.io-client";
import "@fontsource/roboto";

import MultiRobotGraphMap from "./components/graph/MultiRobotGraphMap";
import GraphMap from "./components/graph/GraphMap";
import StartMenu from "./components/setup/StartMenu";

/// socket io constants and instance
const SOCKET = io(window.location.hostname + ":5201");
const SETUP_SOCKET = io(window.location.hostname + ":5202");

export async function fetchWithTimeout(resource, options = {}) {
  const { timeoutMS = 5000 } = options;

  const response = await fetch(resource, {
    ...options,
    signal: AbortSignal.timeout(timeoutMS),
  });

  return response;
}

class VTRGUI extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      // tools menu
      tools_state: {
        annotate_route: false,
      },
      current_tool: null,
      setup_complete: false,
      additional_data: null,
      loading_additional_data: false,
      tried_additional_data: false,
    };
  }

  componentDidMount() {
    // socket io
    SOCKET.on("connect", this.socketConnectCallback.bind(this, true));
    SOCKET.on("disconnect", this.socketConnectCallback.bind(this, false));

    SETUP_SOCKET.on("connect", this.setupSocketConnectCallback.bind(this, true));
    SETUP_SOCKET.on("disconnect", this.setupSocketConnectCallback.bind(this, false));
    SETUP_SOCKET.on("connect_error", this.setupSocketConnectErrorCallback.bind(this));
  }

  componentWillUnmount() {
    // socket io
    SOCKET.off("connect", this.socketConnectCallback.bind(this, true));
    SOCKET.off("disconnect", this.socketConnectCallback.bind(this, false));

    SETUP_SOCKET.off("connect", this.setupSocketConnectCallback.bind(this, true));
    SETUP_SOCKET.off("disconnect", this.setupSocketConnectCallback.bind(this, false));
    SETUP_SOCKET.off("connect_error", this.setupSocketConnectErrorCallback.bind(this));
  }

  // Fetch additional data only after setup is complete, and only try once
  fetchAdditionalData() {
    this.setState({ loading_additional_data: true });
    fetchWithTimeout("/vtr/additional_data", { timeoutMS: 5000 })
      .then((response) => {
        if (response.status !== 200) throw new Error("No additional data " + response.status);
        return response.json();
      })
      .then((data) => {
        this.setState({ additional_data: data, loading_additional_data: false, tried_additional_data: true });
      })
      .catch((error) => {
        this.setState({ loading_additional_data: false, tried_additional_data: true });
        console.error(error);
      });
  }

  render() {
    if (!this.state.setup_complete) {
      return (
        <>
          <StartMenu socket={SETUP_SOCKET} set_setup_complete={this.setSetupComplete.bind(this)} />
        </>
      );
    }
    // After setup is complete, fetch additional data if not already done, and only try once
    if (this.state.setup_complete && this.state.additional_data == null && !this.state.loading_additional_data && !this.state.tried_additional_data) {
      this.fetchAdditionalData();
      return <div>Loading additional data...</div>;
    }
    // If tried and failed, or if data is present, move on
    if (this.state.setup_complete && (this.state.additional_data || this.state.tried_additional_data)) {
      // If we have additional_data and it contains robot_ids, use MultiRobotGraphMap
      if (this.state.additional_data) {
        return (
          <>
            <MultiRobotGraphMap socket={SOCKET} robotIds={this.state.additional_data} />
          </>
        );
      } else {
        // Fallback to single-robot GraphMap
        return (
          <>
            <GraphMap socket={SOCKET}/>
          </>
        );
      }
    }
    return null;
  }

  setSetupComplete(value) {
    this.setState({ setup_complete: value });
  }

  setupSocketConnectErrorCallback() {
    fetch(window.location.hostname + ":5202")
    .then(response => {
      if (response.ok) {
        console.info("Setup server is up, but Socket IO connection failed. Will retry.");
      } else {
        console.info("Setup server is down. Assuming setup has already occurred.");
        this.setState({ setup_complete: true });
        this.setupSocketConnectCallback(false);
      }
    })
    .catch(() => {
      console.info("Setup server is down. Assuming setup has already occurred.");
      this.setState({ setup_complete: true });
      this.setupSocketConnectCallback(false);
    });
  }

  /**
   * @param {boolean} connected Whether or not Socket IO is connected.
   */
  socketConnectCallback(connected) {
    console.info("Socket IO connection state: ", connected);
  }

  setupSocketConnectCallback(connected) {
    console.info("Setup Socket IO connection state: ", connected);
    if (!connected && this.state.setup_complete) {
      console.info("Setup complete, closing connection to Setup Socket IO server.");
      SETUP_SOCKET.disconnect();
    }
  }
}

// ========================================================

ReactDOM.render(
  <React.StrictMode>
    <VTRGUI />
  </React.StrictMode>,
  document.getElementById("root")
);
