import React from "react";
import ReactDOM from "react-dom";
import "./index.css";

import io from "socket.io-client";
import "@fontsource/roboto";

import GraphMap from "./components/graph/GraphMap";

/// socket io constants and instance
const SOCKET = io(window.location.hostname + ":5201");

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
    };
  }

  componentDidMount() {
    // socket io
    SOCKET.on("connect", this.socketConnectCallback.bind(this, true));
    SOCKET.on("disconnect", this.socketConnectCallback.bind(this, false));
  }

  componentWillUnmount() {
    // socket io
    SOCKET.off("connect", this.socketConnectCallback.bind(this, true));
    SOCKET.off("disconnect", this.socketConnectCallback.bind(this, false));
  }

  render() {
    return (
      <>
        <GraphMap socket={SOCKET} />
      </>
    );
  }

  /**
   * @brief Sets the socketConnected state variable based on true Socket IO connection status.
   * @param {boolean} connected Whether or not Socket IO is connected.
   */
  socketConnectCallback(connected) {
    console.info("Socket IO connection state: ", connected);
  }
}

// ========================================================

ReactDOM.render(
  <React.StrictMode>
    <VTRGUI />
  </React.StrictMode>,
  document.getElementById("root")
);
