# vtr_frontend

This folder contains the frontend of VTR user interface.

## Table of Contents

- [vtr_frontend](#vtr_frontend)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
  - [Launch](#launch)
  - [Development](#development)

## Installation

See [here](./vtr-ui/README.md). **TODO**: integrate UI build into CMakeLists.txt.

## Launch

Since VT&R3 is under development, the user interface still relies on VT&R2.1. First launch VT&R2.1 and then

```bash
rosrun vtr_interface ui_server.py /Navigation:=/ShamNav # ShamNav is the default namespace of VTR2.
```

## Development

VTR3 user interface is a single page web app tested with Google Chrome and Firefox. The backend server uses python [Flask](https://flask.palletsprojects.com/en/1.1.x/), and the implementation is [here](../src/vtr_interface/ui_server.py). The frontend is written in html, javascript and css, and it mainly uses the following libraries:

- [React](https://reactjs.org/): A JavaScript library for building user interfaces.
  - Used for designing the entire user interface.
  - We choose this library because it is the most popular one at the moment.
- [Leaflet](https://leafletjs.com/) and [React-Leaflet](https://react-leaflet.js.org/): An open-source JavaScript library for mobile-friendly interactive maps and its React wrapper.
  - Used for displaying the background map and pose graph.
- [Socket.IO](https://socket.io/): Real-time communication.
  - Used for getting real-time updates from the main VT&R process.
- [Protocol Buffers](https://developers.google.com/protocol-buffers): Language-neutral, platform-neutral extensible mechanism for serializing structured data.
  - Used for serializing graph and robot updates.
- [Webpack](https://webpack.js.org/) and [Babel](https://babeljs.io/): JavaScript bundler and compiler, respectively.
  - Used for transforming and optimizing the frontend code so that we can serve it using our backend.
  - We choose these two packages because they are the default of [Create React App](https://create-react-app.dev/), which is used to start our UI development.

Always make sure the packages being used are the latest, upgrade packages to the latest version periodically (`npx npm-check-updates -u`). See README inside `vtr-ui` for more information about how to build the frontend. Currently we do not support using the default ES6 server provided by [Create React App](https://create-react-app.dev/), so unfortunately you have to debug UI with VTR running.
