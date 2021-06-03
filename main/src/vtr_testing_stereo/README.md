# vtr_testing_stereo

Source environment.

```bash
source ${VTRVENV}/bin/activate
source ${VTRSRC}/main/install/setup.bash
```

## Test using the nov4 dataset

Odometry:

```bash
# remove existing runs
rm -r /tmp/graph0.index
# In the first terminal run
ros2 launch vtr_testing_stereo odometry.launch.py
# When the vtr node is ready, press enter.
```

Localization:

we use the same data for localization (basically a sanity check).

Remember to reset the rviz window.

This has to be run after odometry, and you can run it any number of times

```bash
# In the second terminal
ros2 run vtr_lidar test_kitty.py
# In the first terminal run
ros2 launch vtr_testing_lidar localization.launch.py params:=lidar_kitty.yaml
# When the vtr node is ready, press enter.
```
