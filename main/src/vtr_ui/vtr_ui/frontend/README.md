# vtr3_ui

This user interface can but is not meant to run standalone. Include the UI into your projects. For VTR3, follow the installation instructions [there](https://github.com/utiasASRL/vtr3). If you just want to get a sense of how it works, follow the installation instructions below, and choose project "None" at start up.

## Installation

### Dependencies

```bash
sudo apt install -y nodejs npm protobuf-compiler  # system deps
pip3 install flask flask_socketio eventlet python-socketio python-socketio[client] websocket-client  # python deps
```

See [here](./vtr-ui/README.md). **TODO**: integrate UI build into CMakeLists.txt.

### Build the UI

```bash
protoc --python_out=`pwd` *.proto  # build protobuf messages (this is optional since we commit the resulting python scripts, run this command after you make any change to the proto files)
cd vtr-ui && npm install . && npm run build  # install dependencies and then "compile" javascripts
```

## Launch

In one terminal,

```bash
python web_server.py  # web server that hosts the web page and handle some requests from the frontend
```

In another terminal,

```bash
python socket_server.py  # socket io server that passes data between the frontend and other systems, mainly used to send updates from vtr to the frontend
```

Open your brower and navigate to `http://localhost:5200`, and select "None" when choosing project.

## To Developers

VTR3 user interface is a single page web app tested with Google Chrome and Firefox. The backend server uses python [Flask](https://flask.palletsprojects.com/en/1.1.x/), and the implementation is [here](./web_server.py). The frontend is written in html, javascript and css, and it mainly uses the following libraries:

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

Always make sure the packages being used are the latest, upgrade packages to the latest version periodically (`npx npm-check-updates -u`). See README inside `vtr-ui` for more information about how to build the frontend. Currently we do not support using the default ES6 server provided by [Create React App](https://create-react-app.dev/), so you have to debug UI with VTR running.
