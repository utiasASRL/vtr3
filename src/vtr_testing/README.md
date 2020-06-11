# vtr_testing

This package is a top-level package that contains a number of executables, their launch files and params, for use in re-playing robochunk data offline in an easy to use manner. You might like to look at the [tutorial](tutorial.md).

The current executables are:

- *module_vo*: For running stereo VO on data gathered from the ASRL Grizzly
- *module_loc*: For running stereo VO + localisation on data gathered from the ASRL Grizzly and an existing graph
- *lancaster_vo*: For running mono VO on data gathered from the ASRL Lancaster
- *lancaster_loc*: For running mono VO + localisation on data gathered from the ASRL Lancaster and an existing graph

To use these binaries, you need data in robochunk format for the specific offline tool. To run each tool, use the launch files located in the `launch/scenarios/` folder.
For a specific dataset, you might want to use your own param file. These are located in the `param/scenarios/` folder, where you can add your own.

### An example

An example of the command to run one of the offline tools is:
```
roslaunch vtr_testing lancaster_vo_phoenix.launch scenario_params=lancaster_vo_tegra_orb debug:=false
```
The launch files usually (but not in all cases) have a number of configurable input params, such as:
  - *scenario_params*: the name of the params file to use in the `param/scenarios/` folder (don't include the '.yaml')
  - *feature_type*: either SURF or ORB. The default is usually SURF.
  - *robot_type*: grizzly, trex or lancaster. The default is usually specific to the offline binary you are using.
  - *debug*: defaults to false. When debug is true the offline tool won't launch (you'll have to use `rosrun`), but it will load the params and an _rviz_ session specific for the binary.

### Param files
A param file contains all the parameters specific to your computer and the dataset you are running with. It's best to look at and copy one of the other param files in `parm/scenarios/` when making your own. Most params are the same as in *asrl__navigation*'s Navigator param files. The important differences are highlighted here:

 - *input_data_dir*: Where the sensor data lies. This folder usually contains a number of folders titled _run_XXXXXX_.
 - *results_dir*: The folder in which the results module will save data. Not really used anymore.
 - *data_dir*: Where to save the graph. This *could* be the same as _input_data_dir_, but if you don't want to mess up the data there, everything new will be saved in *this* folder location.
 - *sim_run*: The run to use, corresponding to the folder in _input_data_dir_ titled _run_XXXXXX_.
 - *cam_stream_name*/gps_stream_name: The robochunk data sensor stream name corresponding to the data in _run_XXXXXX_.
 - *start_index*: The image index to seek to when starting
 - *stop_index*: The image index to play last before quitting
 - *scale*: How much to scale the input images. 1.0 means no scale. 0.5 means reduced in size by half.
 - *throttle*: If true, limit playback speed to the approximate timestamps of the data. If false, play back as fast as possible, ignoring the data timestamps.

### A Tutorial
For an in depth introduction to VTR2 via the offline tools, see the [tutorial](tutorial.md)
