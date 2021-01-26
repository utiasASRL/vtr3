# ASRL Playback Tool Tutorial

## Launch the entire system

Remember to source the workspace.

Start the whole vtr system

```bash
cd ~/ASRL/vtr3/ros2/src/vtr_navigation/tmuxp
tmuxp load vtr3.yaml
```

See the bottom panel for how to unpause the system and start teach.

Start a replay tool that keeps publishing images from a dataset.

```bash
ros2 run  vtr_sensors BumblebeeReplay ~/ASRL/dataset/20200923_bag2 front_xb3 false
```
