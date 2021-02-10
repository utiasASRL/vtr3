# vtr2.2

This is NOT the ROS1 version of VTR. It is only used for testing while we migrating to ROS2.

- [vtr2.2](#vtr22)
  - [Installation](#installation)
    - [Code Base Overview](#code-base-overview)
    - [Install ROS1](#install-ros1)
    - [Install ros1_bridge](#install-ros1_bridge)
    - [Install catkin tools](#install-catkin-tools)
    - [Install Third-Party ROS1 Packages (VTR2 Build)](#install-third-party-ros1-packages-vtr2-build)
    - [Install utiasASRL robots library](#install-utiasasrl-robots-library)
    - [Install VTR2.1](#install-vtr21)
    - [Clean-Up (This section is not needed for now, because I don't know where the following env vars are used.)](#clean-up-this-section-is-not-needed-for-now-because-i-dont-know-where-the-following-env-vars-are-used)
    - [Install VTR2.2](#install-vtr22)
    - [Prepare VTR2.2 for launching](#prepare-vtr22-for-launching)

## Installation

### Code Base Overview

### Install ROS1

Requied version: Noetic

We install ROS1 under `~/ASRL/workspace/ros_noetic`. Instructions follow the installation tutorial [here](http://wiki.ros.org/noetic/Installation/Source)

Add ROS1 repositories,

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

Install dependencies,

```bash
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
```

Initialize rosdep,

```bash
sudo rosdep init
rosdep update
```

First download necessary ROS packages.

```bash
mkdir -p ~/ASRL/workspace/ros_noetic && cd ~/ASRL/workspace/ros_noetic  # root dir for ROS1
rosinstall_generator desktop_full --rosdistro noetic --deps --tar > noetic-desktop-full.rosinstall
mkdir src
vcs import --input noetic-desktop-full.rosinstall ./src
rosdep install --from-paths src --ignore-src --rosdistro noetic --skip-keys="libopencv-dev python3-opencv" -y
```

- Note: we use `--skip-keys` option to skip installing opencv related tools, since we have already installed them from source.

Install ROS1 via `catkin_make_isolated`

```bash
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space ~/ASRL/workspace/ros_noetic/install
```

**Important**: DO NOT run the following command. Normally you run the following command immediately after installing ROS1 to add path to its executables:

```bash
source ./install/setup.bash  # ROS 1 packages should always extend this workspace.
```

However, since now we also need to install ROS2, **DO NOT** run the above command after installation. If you have run it already, open a new terminal and continue.

VTR2 targets ROS1 while VTR3 targets ROS2. Currently, VTR3 is under active development, so we need to run both VTR2 and VTR3 on the same computer for testing purposes. Therefore, we install both ROS1 and ROS2, and use a ROS2 package called `ros1_bridge` to let ROS1 and ROS2 packages communicate with each other.

### Install ros1_bridge

Now we install the bridge between ROS1 and ROS2.

- The following instructions are based on [here](https://github.com/ros2/ros1_bridge).
- In fact, the instruction is put here only for completeness, you should install this package after you have installed VTR2 and VTR3. See the _important_ message below.

```bash
cd ~/ASRL/workspace/ros_foxy
# Source both ROS1 and ROS2 workspaces
source ~/ASRL/workspace/ros_noetic/install/setup.bash
source ~/ASRL/workspace/ros_foxy/install/setup.bash
# Now install ros1_bridge
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

**Important**: currently `ros1_bridge` only works for the packages from the workspaces sourced by the time it is installed. Therefore, we have to reinstall this package every time we build a new workspace (either ROS1 or ROS2), which means that after we have installed VTR2 and VTR3, we need to rerun the above command again with the last workspace we have created:

```bash
cd ~/ASRL/workspace/ros_foxy
# Source both ROS1 and ROS2 workspaces
source <VTR2 workspace>/setup.bash
source <VTR3 workspace>/setup.bash
# Now install ros1_bridge
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

### Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)

This is a better version of `catkin_make` that is commonly used to build ROS1 packages. We use this tool to build VTR2 and its dependent packages. In ROS2, we use `colcon`, which is the default build tool for ROS2 packages.

```bash
sudo apt-get install python3-catkin-tools python3-osrf-pycommon
```

- Note: `python3-osrf-pycommon` is a dependency of catkin tools but not automatically installed due to a bug in catkin tools 0.5.0. TODO remove this once the bug is fixed.

### Install Third-Party ROS1 Packages (VTR2 Build)

Before continue, start a new terminal and source only the ROS1 workspace

```bash
source ~/ASRL/workspace/ros_noetic/install/setup.bash  # ROS 1 packages should always extend this workspace.
```

We install third-party ROS packages in the following directory

```bash
mkdir -p ~/charlottetown/extras/src
cd ~/charlottetown/extras/src
```

Now follow the instructions to download the repositories relating to your robot.

**The following sections for each robot are optional. You only need to install packages for a particular robot if you are going to use it, or use data that was gathered with that robot.** Don't have a specific robot you're working on? Follow the **grizzly** instructions.

- Grizzly

  Download source code.

  ```bash
  git clone https://github.com/utiasASRL/joystick_drivers.git
  git clone https://github.com/utiasASRL/grizzly.git -b vtr3_development  # tracking our development branch
  ```

  For the `joystick_driver` repository, you need to change the `package.xml` in its `wiimote` package to comment out the `python-cwiid` dependency and change `python-numpy` to `python3-numpy`. We cannot create new branches in this repo so the changes are not published.

  Install dependencies via rosdep for your ROS version

  ```bash
  rosdep install --from-paths ~/charlottetown/extras/src --ignore-src --rosdistro <your ROS distribution>
  ```

  - Note: The above command should not install any package that starts with `ros-` using `apt-get`.

If you downloaded any third party packages in the `extras/src` folder, then install them via `catkin build`

```bash
cd ~/charlottetown/extras
export UTIAS_ROS_DIR=~/charlottetown # not sure why this is needed
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source ~/charlottetown/extras/devel/setup.bash
```

### Install utiasASRL robots library

We install the robots library in the following directory

```bash
mkdir -p ~/charlottetown/utiasASRL/robots
cd ~/charlottetown/utiasASRL/robots
```

Download the repo from github

```bash
git clone https://github.com/utiasASRL/robots.git src -b vtr3_development  # tracking our development branch
```

Install it via `catkin build`

```bash
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source ~/charlottetown/utiasASRL/robots/devel/setup.bash
```

### Install VTR2.1

Install VTR system dependencies

```bash
sudo apt-get install cmake libprotobuf-dev protobuf-compiler libzmq3-dev \
  build-essential libdc1394-22 libdc1394-22-dev libpugixml1v5 libpugixml-dev \
  libgtest-dev
```

We install the vtr library in the following directory

```bash
mkdir -p ~/charlottetown/utiasASRL/vtr2
cd ~/charlottetown/utiasASRL/vtr2
```

Download vtr from github

```bash
git clone https://github.com/utiasASRL/vtr2.git src -b vtr3_development  # tracking our development branch
cd ~/charlottetown/utiasASRL/vtr2/src
# Submodule update the UI and deps.
git submodule update --init --remote  # add remote flag so that git automatically track the latest updates of the submodules.
```

**Important** `vtr3_development` branch has the following changes to support ubuntu >=16.04:

- Added the following two lines to the CMakeLists.txt of all vtr2 packages that contain the line `find_package(Eigen3 3.x.x REQUIRED)` (there are 12 total - 10 in asrl\_\_\* packages as well as LGmath and STEAM). We are unsure yet if this is needed on 20.04.

  ```bash
  add_definitions(-DEIGEN_DONT_VECTORIZE=1)
  add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)
  ```

  This degrades performance but prevents [Eigen alignment issues](http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html). Why this is an issue on some systems but not others is unknown.

- For the submodule gpusurf library, set the correct compute capability for Lenovo P53 GPU. You can look for it [here](https://developer.nvidia.com/cuda-gpus). `gpusurf/CMakeLists.txt` line 53, change _7.5_ to the version of CUDA you are using (e.g. _10.2_). On line 55, change the _compute_30_ and _sm_30_ values to the value on the nvidia webpage (minus the '.') (e.g. 7.5 becomes _compute_75_ and _sm_75_) and remove "_sm_50_".
- For the submodule robochunk, changed robochunk build type to debug in this file `~/charlottetown/utiasASRL/vtr2/src/deps/catkin/.catkin_tools/profiles/default/config.yaml`
- A lots small changes needed to get VTR2 compile on Ubuntu 18.04 and 20.04, which has a newer gcc/g++ version.

Now go to `deps` dir and install vtr dependencies (including robochunk)

```bash
cd ~/charlottetown/utiasASRL/vtr2/src/deps/catkin
catkin build
source ~/charlottetown/utiasASRL/vtr2/devel/deps/setup.bash
```

Install python dependencies

We install all python dependencies inside a python virtual environment so that they do not corrupt system python packages. **Important**: activate the virtualenv whenever you install things starting from this stage and run vtr.

```bash
sudo apt install python3-virtualenv
cd ~/ASRL && virtualenv venv --system-site-packages
source ~/ASRL/venv/bin/activate
pip install pyyaml pyproj scipy zmq socketIO_client flask flask_socketio
```

Install protobuf python packages via `pip`.

There are 3 in total.

```python
cd ~/charlottetown/utiasASRL/vtr2/devel/deps/.private/robochunk_msgs/robochunk_protopy
pip install -e .
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/src/robo_sensors/setup/babelfish_non_primitive_protopy
pip install -e .
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/src/robo_sensors/rtp/babel_rtp_robochunk_protopy
pip install -e .
```

Build robochunk translator (currently this has to be built after building the libs in `deps`)

```bash
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/
catkin init
catkin config --no-cmake-args
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Debug  # Note: A bug in robochunk, there is a confusing runtime error when setting build type to Release.
catkin build
source ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/devel/setup.bash
```

- Note: it is weird that the original installation instruction did not source the setup.bash file of the extended translator workspace. Not sure why.

Same as above, install protobuf python packages.

```python
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/devel/.private/babelfish_non_primitive/babelfish_non_primitive_protopy
pip install -e .
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/devel/.private/babel_rtp_robochunk/babel_rtp_robochunk_protopy
pip install -e .
```

Build VTR

```bash
cd ~/charlottetown/utiasASRL/vtr2/src/
catkin build
source ~/charlottetown/utiasASRL/vtr2/devel/repo/setup.bash
```

**Important**: Currently there's a [bug](https://github.com/protocolbuffers/protobuf/issues/1491) with the generated python protobuf package (relative/absolute import). The current workaround is to manually change absolute import to relative import used in each python file generated from protobuf, through the `sed` command `sed -i -r 's/^import (.*_pb2)/from . import \1/g' *_pb2*.py`. This is a ugly hack but should be enough for now since we won't be using robochunk in vtr3.

```bash
cd ~/charlottetown/utiasASRL/vtr2/devel/deps/.private/robochunk_msgs/robochunk_protopy/robochunk/proto
sed -i -r 's/^import (.*_pb2)/from . import \1/g' *_pb2*.py
cd ~/charlottetown/utiasASRL/vtr2/src/asrl__messages/src/asrl__messages/proto
sed -i -r 's/^import (.*_pb2)/from . import \1/g' *_pb2*.py
sed -i -r 's/as .*_pb2 as/as/g' *_pb2*.py
```

- Note: There should be more protobuf generated scripts that need the above change, but I haven't found them all. If you encounter any python error that looks like: `Cannot import ...`, it should be the above problem.

### Clean-Up (This section is not needed for now, because I don't know where the following env vars are used.)

We are going to set up a more permanent source for the 1 workspace we have set up (ROS).

Open the bashrc file

```bash
gedit ~/.bashrc
```

Add the following to your bashrc file

```bash
###ASRL-ROS
#Set the ASRL Code directory:
export ASRL_CODE_DIR=~/asrl-misc-code

#Set the charlottetown root folder, it is assumed that all workspaces (ros_osrf, extras, utias, YOUR_LAB) and the scripts/rosinstalls/logs reside in this folder
export UTIAS_ROS_DIR=~/charlottetown

#Set the name of your lab workspace
export UTIAS_LAB_WS=asrl

#Add the helper scripts to the PATH:
export PATH="${UTIAS_ROS_DIR}"/scripts/:$PATH

#Set the data logging directory:
export ASRL_DATA_DIR="${UTIAS_ROS_DIR}"/logs

#Source the vtr2 workspace (which includes all parents)
. ~/charlottetown/utiasASRL/vtr2/devel/repo/setup.bash
#Announce the currently sourced version
echo "Sourced: ${UTIAS_ROS_DIR}"

# Catkin recursive cmd in src
function catkin_src_cmd () {
    test $# -lt 1 && echo "Must provide a command to run!" && return 1
    test -d src || echo "There is no 'src' subdirectory!"
    test -d src || return 1
    local command="cd {} && echo \"--- In: {} ---\" && $@"
    find ./src -mindepth 1 -maxdepth 1 -type d -exec sh -c "$command" \;
}
```

You are finished installing VTR2! You should now take a look at **asrl\_\_navigation** and **asrl\_\_offline_tools** and their top-level READMEs. To verify your installation is working and to get started with running VTR2, follow the follow the [First Run Tutorial](https://github.com/utiasASRL/vtr2/blob/develop/asrl__offline_tools/FirstRunTutorial.md) in [asrl\_\_offline_tools](https://github.com/utiasASRL/vtr2/tree/develop/asrl__offline_tools).

### Install VTR2.2

**Note**

1. VTR2.2 is not supposed to be usable. It only contains the ros1 version of vtr packages that will soon be ported to ros2.
2. For now, you must install VTR2.1 before installing VTR2.2.

Install VTR2.2 dependencies.

```
sudo apt install doxygen  # For building the documentations.
sudo apt install nodejs npm  # For building the user interface.
```

Clone this repo and then build it with Catkin.

```bash
cd ~/ASRL
git clone https://github.com/utiasASRL/vtr3.git
cd vtr3
git submodule update --init --remote  # For rosbag2+vtr_storage in vtr3.
cd ros1
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
catkin build --catkin-make-args run_tests  # Build and run tests, note that you must run "catkin build" before this command.
catkin_test_results build  # Ensure that all unit tests pass.
source ~/ASRL/vtr3/ros1/devel/setup.bash
```

### Prepare VTR2.2 for launching

After installing VTR2.2, either add the following commands to `bashrc` or run them everytime before launching VTR3.

```bash
source ~/ASRL/venv/bin/activate  # Enter the venv we created for VTR3.
source ~/ASRL/vtr3/ros1/devel/setup.bash  # source the ros setup.
source $(rospack find vtr_navigation)/setup.bash  # define environment variables for VTR3.
```

You are finished installing VTR2.2. Now take a look at the documentations and tutorials below on how to use it.
