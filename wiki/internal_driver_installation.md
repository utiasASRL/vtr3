Before installing our own drivers and robot description packages, you should first install all VT&amp;R3 dependencies by following the [installation guide](./installation_guide.md) up until the `build and install driver and robot description packages` section.

Download, build and install drivers and robot description packages for ASRL internal use.

```bash
# install system dependencies
sudo apt install -y libdc1394-22 libdc1394-22-dev  # for BumbleBee stereo camera
sudo apt install -y libbluetooth-dev libcwiid-dev  # for joystick drivers

# download drivers source code from our private repository
mkdir -p ${VTRDEPS}/vtr3_drivers && cd $_
git clone git@github.com:utiasASRL/vtr3_drivers.git .
git submodule update --init --remote

# source the ROS2 workspace with necessary dependencies installed
source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash

# build and install drivers
cd ${VTRDEPS}/vtr3_drivers/ros2
colcon build --symlink-install
source ${VTRDEPS}/vtr3_drivers/ros2/install/setup.bash

# build and install robot description packages
cd ${VTRSRC}/robots/ros2
colcon build --symlink-install
source ${VTRSRC}/robots/ros2/install/setup.bash
```

Now you should proceed to install VT&amp;R3 packages and add-ons.

```bash
# source the ROS2 workspace with all VTR3 dependencies installed (including drivers and robot description packages)
source ${VTRSRC}/robots/ros2/install/setup.bash

# build and install VTR3 packages
cd ${VTRSRC}/main
colcon build --symlink-install

# build VTR3 web-based GUI
VTRUI=${VTRSRC}/main/src/vtr_ui/vtr_ui/frontend/vtr-ui
npm --prefix ${VTRUI} install ${VTRUI}
npm --prefix ${VTRUI} run build

# source the ROS2 workspaces with VTR3, drivers and robot description packages installed
source ${VTRSRC}/main/install/setup.bash

# build and install add-ons (this includes vtr_bumblebee_xb3, which is necessary to run VTR3 on Grizzly)
cd ${VTRSRC}/extra
colcon build --symlink-install
```
