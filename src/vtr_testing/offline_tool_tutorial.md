# Offline Tool Tutorial

This tutorial presents a step-by-step guide to running Visual Odometry (VO) and/or Localization using the offline tools. This will help verify that your VTR installation is working correctly and provide a first introduction to using VTR. The examples in this tutorial are not the full VTR system, but simpler binaries that we use to test new parameters or configurations offline on datasets that we already have.

## Table of Contents

- [Offline Tool Tutorial](#offline-tool-tutorial)
   - [Table of Contents](#table-of-contents)
   - [Download and examine an example dataset](#download-and-examine-an-example-dataset)
   - [Module VO](#module-vo)
      - [Modify the scenario file](#modify-the-scenario-file)
      - [Run Module VO](#run-module-vo)
      - [Examine the Output](#examine-the-output)
   - [Module Loc](#module-loc)
      - [Run Module VO to generate teach runs](#run-module-vo-to-generate-teach-runs)
      - [Modify scenario parameters](#modify-scenario-parameters)
      - [Run Module Loc](#run-module-loc)

## Download and examine an example dataset

Download the dataset from [Google Drive](https://drive.google.com/drive/folders/1zc4E1iJfY9wrEWbWM25qi-Y4TPtxhGuz?usp=sharing) into appropriate location on your computer.

All data are in robochunk format. You will see a number of `run_00000X` folders. Each of these folders contains all the images and GPS data recorded on the robot (grizzly) for a _run_, which is a contiguous set of data where the vehicle travered some trajectory from point A to B. The images are stored in 'chunks' of a few GB in size. You can see one in `run_000000/sensorData/front_xb3/chunk000000/` simply called _data_. Also in this folder is a _calibration_ file that stores the stereo calibration, a _header_ file that describes the data type and a folder of _indexFiles_ that allows the robochunk API to perform random access on the data based on timestamps or other indexes.

By convention, all data associated with a sensor is stored in the `sensorData` folder of a run, including its products (which might be features or landmarks in the case of images), under a subfolder name associated with the sensor name. In this case `front_xb3` refers to the Point Grey XB3 stereo camera mounted on the front of the Grizzly. The data is also stored under a name referring to its type. In this case `processed_stereo_images` implies that the data is a set of stereo images that have had some processing applied.

## Module VO

### Modify the scenario file

VO parameters are specific to a particular set of data. We store all this information (for the binaries to load) in a _scenario file_, which is a yaml file that gets loaded by ROS as a param file. Here we will make a new file specific to your setup to run ModuleVO.

1. Copy _module\_vo\_grizzly\_surf.yaml_ in `vtr_testing/param/scenarios/` and rename it to _my\_module\_vo\_test.yaml_ or something similar. Open the file and perform the following actions.
2. Change the directory pointed to by _input\_data\_dir_ to the top level directory that contains the `run_00000X` folders. This variable points to where the input dataset in robochunk format is stored.
3. Make a results directory and change the _results_dir_ variable to point to this location. In most cases this will not be filled with any data, but other offline tools may do so.
4. Make a folder to store the output data in, then change _data\_dir_ to point to this location. This will hold the robochunk products that ModuleVO will produce and save. This doesn't have to be different from _input_data_dir_, because it saves data in the same directory heirarchy, but if you want to keep your input data directory clean, then it's best to make this a different folder.
5. Change _sim\_run_ to **'/run_000000'**. This means ModuleVO will process the stereo imagery stored in the folder corresponding to run 0. You can change this variable later to point to any of the runs in the input dataset, and ModuleVO will process that run instead. Play around with these runs later.
6. Change _start\_index_ to **1**. This means module VO will seek to the first index point and process the data sequentially from there. You can change the start point in processing by changing this number to a higher value. If the value is beyond the end of the dataset or larger than _stop\_index_ (the last index that ModuleVO will process before quitting), ModuleVO will simply quit straight away.
7. Check that _quick\_vo/mono\_ransac/visualize\_ransac\_inliers_ is set to **true**. This means that you will see a window with the processed VO feature tracks when you run ModuleVO. You can set some of the other parameters that contain the word 'visualize' to true to see some of the results of intermediate steps. This is useful for debugging later. However, the more visualisations you have, the slower VO will run.
8. Save the file. This is where you can store all the non-default parameters that are specific to your dataset and setup. You will likely have one of these files to store all your parameters wheneer you use a different configuration of robot or test in a new environment.

### Run Module VO

Examine the ROS launch file we will be using to run ModuleVO located at _vtr\_testing/launch/scenarios/module\_vo\_grizzly.launch_
There are a number of input parameters with sensible defaults. You can change these input parameters on launch to suit your configuration. In this case, we will need to specify the _scenario\_params_ to point to the scenario params file you just made (but drop the .yaml extenson). You can now run Module VO.

In a terminal, run ModuleVO by executing the launch file:

```bash
roslaunch vtr_testing module_vo_grizzly.launch scenario_params:=<my\_module\_vo\_test>
```

If everything is configured correctly, you should see some startup debugging output in the terminal and a window open showing the VO tracks. If everything is going well, there should be occasional updates in the terminal showing the processing rate.

When the end of the run is reached, the window will close and ModuleVO will exit. You may need to Ctrl-C the roslaunch process manually to drop back to a regular prompt.

### Examine the Output

There should now be data stored in the directory you specified in the _data\_dir_ variable of your scenario file. 
If you have run ModuleVO for the first time with the _sim\_run_ variable set to **'/run_000000'**, you'll see this folder in the output _data\_dir_. 
Inside `run_000000/sensorData/front_xb3`, you will see `observations`, `landmarks` and `visualization_images` directories, referring to the output products of ModuleVO. 
At the top level of _data\_dir_, you will also see _graph0.index_, which contains an index to each of the runs. In this case, it will only refer to run 0.

Python scripts in VTR2.1 can be used to visualize the results.
In a terminal, run the following command to plot the integrated VO, where _path/to/data_ is replaced with your _data\_dir_:

```bash
python ~/charlottetown/utiasASRL/vtr2/src/asrl__analysis/examples/plot_vo.py -p /path/to/data
```

You can now process the other runs by changing the _sim\_run_ variable and exploring some of the other parameters in your scenario file.

## Module Loc

### Run Module VO to generate teach runs

1. To run localization offline, we require multiple runs of the same path.
   Start by downloading `run_000001` and `run_000003` from the master In the Dark dataset on [Speculatrix](http://192.168.42.2/das/ASRL/2016-In_The_Dark/master/).
   Runs 6, 9, and 12 are also good ones to start with from this dataset if you would like to try multi-experience localization.

2. Change the directory pointed to by _input\_data\_dir_ in your ModuleVO parameter file (e.g. _my\_module\_vo\_test.yaml_) to the top level directory that contains the In the Dark `run_00000X` folders.
   Make a folder to store the output data in, then change _data\_dir_ to point to this location.
   Change _sim\_run_ to **'run_000001'**.

3. Run ModuleVO by executing the launch file:

   ```bash
   roslaunch vtr_testing module_vo_grizzly.launch scenario_params:=<my\_module\_vo\_test>
   ```

   This generates a teach path for us to localize against.
   Note: though we are processing `run_000001` from the In the Dark set, by default it will be saved as `run_000000` in the output directory.

### Modify scenario parameters

1. Copy _module\_loc\_grizzly\_surf.yaml_ in `vtr_testing/param/scenarios/` and rename it to _my\_module\_loc\_test.yaml_ or something similar.
   Open the file in a text editor.

2. Change both _input\_data\_dir_ and _data\_dir_ to point to the same locations as were used for ModuleVO.
   The new run to be processed will be taken from _input\_data\_dir_.
   The processed teach run will be taken from _data\_dir_ and used to repeat.
   The result will also be stored in _data\_dir_.
   Change _sim\_run_ to **'/run_000003'**.
   Again, though we are processing `run_000003` from the In the Dark set, by default it will be saved as `run_000001` in the output directory.

3. Set _save\_graph_ to true.
   This will add the localization results to the graph as another experience to be used in multi-experience localization and allow the results to be plotted.
   Note: if you wish to run ModuleLoc again on the same `run_00000X` you will need to either delete the previously processed `run_00000X` or use a different _data\_dir_ to avoid duplicated Persistent IDs in the graph.

### Run Module Loc

In a terminal, run ModuleLocalization by executing the launch file:

```bash
roslaunch vtr_testing module_loc_grizzly.launch scenario_params:=<my\_module\_loc\_test>
```

If everything is configured correctly, you should see some startup debugging output in the terminal and the same windows as earlier open showing the VO tracks. 
There should also be a window showing the localization results.

If everything is going well, there should be occasional updates in the terminal showing optimization statistics.

Again, Python scripts in VTR2.1 can be used to visualize the results.
In a terminal, run the following command to plot the integrated VO of the teach pass and the localized repeat paths, where _path/to/data_ is replaced with your _data\_dir_:

```bash
python ~/charlottetown/utiasASRL/vtr2/src/asrl__analysis/examples/plot_loc.py -g /path/to/data
```

You can now process the other runs by changing the _sim\_run_ variable and exploring some of the other parameters in your scenario file.