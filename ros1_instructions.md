# ROS1 Instructions

The following (additional) installation instructions are for cases such as a dataset needs to be converted from ros1 to ros2, or a robot is not ros2 enabled.

## Installation

### Install [ROS Noetic](https://www.ros.org/)

Instructions follow the installation tutorial [here](http://wiki.ros.org/noetic/Installation/Source).

Add ROS1 repositories:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

Install dependencies:

```bash
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
```

Initialize rosdep:

```bash
sudo rosdep init
rosdep update
```

First download necessary ROS packages:

```bash
mkdir -p ${VTRDEPS}/ros_noetic && cd ${VTRDEPS}/ros_noetic  # root dir for ROS1
rosinstall_generator desktop_full --rosdistro noetic --deps --tar > noetic-desktop-full.rosinstall
mkdir -p src
vcs import --input noetic-desktop-full.rosinstall ./src
rosdep install --from-paths src --ignore-src --rosdistro noetic --skip-keys="libopencv-dev python3-opencv" -y
```

- Note: use the `--skip-keys` option to skip installing opencv related tools, since we should have already installed them from source.

Install ROS via `catkin_make_isolated`:

```bash
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space ${VTRDEPS}/ros_noetic/install
```

Source the installation

```bash
source ${VTRDEPS}/ros_noetic/install/setup.bash  # ROS 1 packages should always extend this workspace.
```

### Install ros1_bridge

The following instructions are based on [here](https://github.com/ros2/ros1_bridge).

Start a terminal without sourcing any of ROS1/ROS2 workspaces.

Source both ROS1 and ROS2 workspaces

```bash
source ${VTRDEPS}/ros_noetic/install/setup.bash
source ${VTRDEPS}/ros_foxy/install/setup.bash
```

Download source code

```bash
mkdir -p ${VTRDEPS}/ros1_bridge/src && cd ${VTRDEPS}/ros1_bridge/src
git clone git@github.com:ros2/ros1_bridge.git
cd ros1_bridge
git checkout foxy
cd ${VTRDEPS}/ros1_bridge
colcon build --symlink-install
```

Source the installation

```bash
source ${VTRDEPS}/ros1_bridge/install/setup.bash
```
