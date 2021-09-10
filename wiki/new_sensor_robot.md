## Setup VT&amp;R3 with a New Sensor or Robot

### (TODO: Sherry, Yuchen) Sensor

Stereo camera

- expected stereo image input format (vtr customized ros2 message `rig_images`).
- example on how to convert stereo-camera images to our message format
- what topic vtr listens to (`xb3_images`) and how to change it

LiDAR

- expected input format (sensor_msgs/PointCloud2), what fields are needed (`xyzt` where `t` is point-wise time stamp)
- what topic vtr listens to (`points`) and how to change it

### (TODO: Yuchen, Jordy) Robot

- how to use xacro to specify frames: sensor frame, control frame, etc
