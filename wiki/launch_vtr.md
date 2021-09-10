## (TODO: Jordy) Launch VT&amp;R3

We use [tmux](https://github.com/tmux/tmux/wiki) and [tmuxp](https://github.com/tmux-python/tmuxp) to launch and run VT&amp;R3. They are useful tools that allow multiple terminal sessions to be launched and accessed simultaneously in a single window.

### Offline Mode

With the datasets above, VT&amp;R3 can be run without connecting to any sensor or robot.

#### Stereo SURF-Feature-Based T&R

Prerequisites

- Installed VT&amp;R3+UI inside `main` folder including all its dependencies.
- Installed Grizzly robot description packages inside `robots` folder.
- Installed `pgr_triclops` package inside `drivers/ros2` folder and `vtr_bumblebee_xb3` package inside `extra` folder.
- Downloaded the `utias_20210412_camera_vtr_storage` dataset into `${VTRDATA}`.

The video demo [at this link](https://drive.google.com/file/d/12OkMyHeFCCE6MkJDMRz89GfIJI7xFtJA/view?usp=sharing) shows how to

- Launch VT&amp;R3 to use stereo camera images as input.
  ```bash
  tmuxp load ${VTRSRC}/launch/offline_vtr_camera.launch.yaml
  ```
- Use the UI to start teaching a path.
  ```bash
  # replay the first image sequence
  source ${VTRSRC}/extra/install/setup.bash
  # ros2 run vtr_bumblebee_xb3 BumblebeeReplay <dataset directory>                          <topic>   <manual scrub> <start> <end> <replay rate> <sequence>
  ros2 run vtr_bumblebee_xb3 BumblebeeReplay   ${VTRDATA}/utias_20210412_camera_vtr_storage front_xb3 false          900     3500  1             0
  ```
- Use the UI to perform loop closure, i.e., merge into existing path.
- Use the UI to align the graph with the underlying satellite map.
- Use the UI to place the robot on a different location in the graph.
- Use the UI to specify a repeat path and start repeating the path.
  ```bash
  # replay the second image sequence
  source ${VTRSRC}/extra/install/setup.bash
  # ros2 run vtr_bumblebee_xb3 BumblebeeReplay <dataset directory>                          <topic>   <manual scrub> <start> <end> <replay rate> <sequence>
  ros2 run vtr_bumblebee_xb3 BumblebeeReplay   ${VTRDATA}/utias_20210412_camera_vtr_storage front_xb3 false          900     3500  1             1
  ```
- Terminate VT&amp;R3 (`Ctrl-C` once in terminal)

#### LiDAR Point-Cloud-Based T&R

Prerequisites

- Installed VT&amp;R3+UI inside `main` folder including all its dependencies
- Installed Grizzly robot description packages inside `robots` folder
- Downloaded the `utias_20210812_lidar_rosbag` dataset into `${VTRDATA}`.

The video demo [at this link](https://drive.google.com/file/d/19xbPbynoPbpjamt2RJ0-OfQGhCUmqRqE/view?usp=sharing) shows how to

- Launch VT&amp;R3 to use LiDAR point-clouds as input
  ```bash
  # launch command
  tmuxp load ${VTRSRC}/launch/offline_vtr_lidar.launch.yaml
  ```
- Use the UI to start teaching a path and replay point-clouds from the dataset
  ```bash
  # replay the first rosbag
  source ${VTRSRC}/main/install/setup.bash
  ros2 bag play ${VTRDATA}/utias_20210812_lidar_rosbag/rosbag2_2021_08_12-20_02_12
  ```
- Use the UI to perform loop closure, i.e., merge into existing path.
- Use the UI to align the graph with the underlying satellite map.
- Use the UI to place the robot on a different location in the graph.
- Use the UI to specify a repeat path and start repeating the path.
  ```bash
  # replay the second rosbag
  source ${VTRSRC}/main/install/setup.bash
  ros2 bag play ${VTRDATA}/utias_20210812_lidar_rosbag/rosbag2_2021_08_12-20_14_20
  ```
- Terminate VT&amp;R3 (`Ctrl-C` once in terminal)

### (INTERNAL) Online Mode - VT&amp;R3 on Grizzly

#### Grizzly Connection and Control

Perform the following steps to start driving Grizzly manually. Steps marked with "\*" should be done with help from a lab mate during first attempt.

- \*Turn on Grizzly and [configure Grizzly network](https://docs.google.com/document/d/1cEl4yywMoPAmXSkztuviJ0WbO6gnxGhJlt47_H44oBc/edit?usp=sharing).
- \*Connect Grizzly computer (via the ethernet cable) and XBox controller (via usb) to your laptop.
- Use [grizzly_control.launch.yaml](./launch/grizzly_control.launch.yaml) to start a tmux session that passes commands from the XBox controller to Grizzly, by opening a new terminal and inputing the following command. \*You will be asked to input Grizzly computer's password at the bottom pane. Ask a lab mate for password and controller key mappings.
  ```bash
  tmuxp load ${VTRSRC}/launch/grizzly_control.launch.yaml
  ```

You should now be able to drive Grizzly manually (when emergency stop is released). Always keep the tmux session (launched by [grizzly_control.launch.yaml](./launch/grizzly_control.launch.yaml)) on while running (Stereo/LiDAR) VT&R3.

How it works: your machine receives commands from the XBox controller and publishes them as ROS2 messages. The Grizzly computer receives ROS2 messages and converts them to ROS1 messages, which are then received by the internal micro-controller.

#### Stereo SURF-Feature-Based T&R

Connect the Bumblebee XB3 camera (via usb-c) to your machine. (You may be asked to authenticate this connection.) Use [online_vtr_camera.launch.yaml](./launch/online_vtr_camera.launch.yaml) to launch VT&amp;R3 and wait for a few seconds. The web UI should start automatically, and you should see left&right images received from the camera.

```bash
tmuxp load ${VTRSRC}/launch/online_vtr_camera.launch.yaml
```

You should now be able to use VT&R3 same as in offline mode with datasets.

#### LiDAR Point-Cloud-Based T&R

Connect the Honeycomb LiDAR (ethernet-usb) to your machine. Use `honeycomb.launch.yaml` to turn on the sensor, which will then publish point-clouds as ROS1 and ROS2 messages (ros1_bridge is launched alongside).

```bash
tmuxp load ${VTRSRC}/launch/honeycomb.launch.yaml
```

Use `online_vtr_lidar.launch.yaml` to launch VT&amp;R3. The web UI should start automatically.

```bash
tmuxp load ${VTRSRC}/launch/online_vtr_lidar.launch.yaml
```

You should now be able to use VT&R3 same as in offline mode with dataset.
