# vtr_testing_lidar

Source environment.

```bash
source ~/ASRL/venv/bin/activate
source ~/ASRL/vtr3/ros2/install/setup.bash
```

Launch the tmuxp session, rviz2 will auto open.

```bash
tmuxp load /home/yuchen/ASRL/lidar/vtr_lidar/src/vtr_testing_lidar/tmuxp/vtr_testing_lidar.yaml
```

## Test using the kitti dataset

Odometry:

```bash
# remove existing runs
rm -r /tmp/graph0.index
# In the second terminal
ros2 run vtr_lidar test_kitti.py
# In the first terminal run
ros2 launch vtr_testing_lidar odometry.launch.py params:=lidar_kitti.yaml
# When the vtr node is ready, press enter.
```

Localization:

we use the same data for localization (basically a sanity check).

Remember to reset the rviz window.

This has to be run after odometry, and you can run it any number of times

```bash
# In the second terminal
ros2 run vtr_lidar test_kitti.py
# In the first terminal run
ros2 launch vtr_testing_lidar localization.launch.py params:=lidar_kitti.yaml
# When the vtr node is ready, press enter.
```

Test using the electric sheep dataset.

Odometry:

```bash
# remove existing runs
rm -r /tmp/graph0.index
# In the seond terminal
ros2 run electric_sheep PCReplay /home/yuchen/ASRL/dataset/elec_sheep data0408 false 18400 9999999 2 0 2  # use data0408, starting from 18400 scan, without manual scrub
# In the first terminal run
ros2 launch vtr_testing_lidar odometry.launch.py params:=lidar_elecsheep.yaml
# When the vtr node is ready, press enter.
```

Localization:

we use the same data for localization (basically a sanity check).

Remember to reset the rviz window.

This has to be run after odometry, and you can run it any number of times

```bash
# In the second terminal
ros2 run electric_sheep PCReplay /home/yuchen/ASRL/dataset/elec_sheep data0408 false 18400 9999999 2 0 2  # or 26800 for a different run
# In the first terminal run
ros2 launch vtr_testing_lidar localization.launch.py params:=lidar_elecsheep.yaml
# When the vtr node is ready, press enter.
```
