# First Run Tutorial

This tutorial presents a step-by-step guide to running a simple Visual Odometry (VO) example using the offline tools. This will  help verify that your VTR2 installation is working correctly and provide a first introduction to using VTR2. The examples in this tutorial are not the full VTR2 system, but simpler binaries that we use to test new parameters or configurations offline on datasets that we already have.

In this tutorial we will describe the basic robochunk dataset format, show how to configure a param file for a particular scenario and run VO on a short dataset gathered from a stereo camera on the ASRL Grizzly. Let's go!

## Download and examine an Example Dataset

First, you will need a robochunk dataset, which you can get from our [Google Drive](https://drive.google.com/drive/folders/1zc4E1iJfY9wrEWbWM25qi-Y4TPtxhGuz?usp=sharing). Download this entire folder to an appropriate place on your local machine with VTR2 already installed.

All the data in this folder is in robochunk format. We will gloss over some of the details, but will give a general overview of the structure. In this folder you will see a number of `run_00000X` folders.  Each of these folders contains all the images and GPS data recorded on the grizzly for a _run_, which is a contiguous set of data where the vehicle travered some trajectory from point A to B. The images are stored in 'chunks' of a few GB in size. You can see one in `run_000000/sensorData/front_xb3/chunk000000/` simply called _data_. Also in this folder is a _calibration_ file that stores the stereo calibration, a _header_ file that describes the data type and a folder of _indexFiles_ that allows the robochunk API to perform random access on the data based on timestamps or other indexes.

By convention, all data associated with a sensor is stored in the `sensorData` folder of a run, including its products (which might be features or landmarks in the case of images), under a subfolder name associated with the sensor name. In this case `front_xb3` refers to the Point Grey XB3 stereo camera mounted on the front of the Grizzly. The data is also stored under a name referring to its type. In this case `processed_stereo_images` implies that the data is a set of stereo images that have had some processing applied.

## Modify the scenario file

Where the robochunk data is stored is specific to your computer and the VO parameters are specific to a particular set of data. We store all this information (for the binaries to load) in a _scenario file_, which is a yaml file that gets loaded by ROS as a param file. Here we will make a new file specific to your setup to run ModuleVO. Copy *module_vo_grizzly_surf.yaml* in `vtr_testing/param/scenarios/` and rename it to _my_module_vo_test.yaml_ or something similar. Open the file and perform the following actions:

1. Change the directory pointed to by *input_data_dir* to the top level directory that contains the `run_00000X` folders. This variable points to where the input dataset in robochunk format is stored.
2. Make a results directory and change the _results_dir_ variable to point to this location. In most cases this will not be filled with any data, but other offline tools may do so.
3. Make a folder to store the output data in, then change *data_dir* to point to this location. This will hold the robochunk products that ModuleVO will produce and save. This doesn't have to be different to _input_data_dir_, because it saves data in the same directory heirarchy, but if you want to keep your input data directory clean, then it's best to make this a different folder.
4. Change *sim_run* to **'/run_000000'**. This means ModuleVO will process the stereo imagery stored in the folder corresponding to run 0. You can change this variable later to point to any of the runs in the input dataset, and ModuleVO will process that run instead. Play around with these runs later.
5. Change *start_index* to **1**. This means module VO will seek to the first index point and process the data sequentially from there. You can change the start point in processing by changing this number to a higher value. If the value is beyond the end of the dataset or larger than _stop_index_ (the last index that ModuleVO will process before quitting), ModuleVO will simply quit straight away.
6. Check that _quick_vo/mono_ransac/visualize_ransac_inliers_ is set to **true**. This means that you will see a window with the processed VO feature tracks when you run ModuleVO. You can set some of the other parameters that contain the word 'visualize' to true to see some of the results of intermediate steps. This is useful for debugging later. However, the more visualisations you have, the slower VO will run.
7. Save the file. This is where you can store all the non-default parameters that are specific to your dataset and setup. You will likely have one of these files to store all your parameters wheneer you use a different configuration of robot or test in a new environment.

## Run Module VO

Examine the ROS launch file we will be using to run ModuleVO located at *vtr_testing/launch/scenarios/module_vo_grizzly.launch*
There are a number of input parameters with sensible defaults. You can change these input parameters on launch to suit your configuration. In this case, we will need to specify the _scenario_params_ to point to the scenario params file you just made (but drop the .yaml extenson). You can now run Module VO.

In a terminal, run ModuleVO by executing the launch file:

```bash
roslaunch vtr_testing module_vo_grizzly.launch scenario_params=my_module_vo_test
```

If everything is configured correctly, you should see some startup debugging output in the terminal and a window open showing the VO tracks. If everything is going well, there should be occasional updates in the terminal showing the processing rate.

When the end of the run is reached, the window will close and ModuleVO will exit. You may need to Ctrl-C the roslaunch process manually to drop back to a regular prompt.

## Examine the Output

There should now be data stored in the directory you specified in the *data_dir* variable of your scenario file. If you have run ModuleVO for the first time with the *sim_run* variable set to **'/run_000000'**, you'll see this folder in the output _data_dir_. Inside ```run_000000/sensorData/front_xb3```, you will see ```observations```, ```landmarks``` and ```visualization_images``` directories, referring to the output products of ModuleVO. At the top level of *data_dir*, you will also see *graph0.index*, which contains an index to each of the runs. In this case, it will only refer to run 0.

You can now process the other runs by changing the _sim_run_ variable and exploring some of the other parameters in your scenario file.

## Run Module Localization
*Note: ModuleLoc not yet running in VTR3. Instructions are applicable to VTR2.1.*

1. To run localization offline, we require multiple runs of the same path. 
Start by downloading `run_000001` and `run_000003` from the master In the Dark dataset on [Speculatrix](http://192.168.42.2/das/ASRL/2016-In_The_Dark/master/).
Runs 6, 9, and 12 are also good ones to start with from this dataset if you would like to try multi-experience localization.

2. Next, change the directory pointed to by *input_data_dir* in _my_module_vo_test.yaml_ to the top level directory that contains the In the Dark `run_00000X` folders.
Make a folder to store the output data in, then change *data_dir* to point to this location.
Change *sim_run* to **'/run_000001'**.
 
3. Run ModuleVO following the previous instructions. 
This generates a teach path for us to localize against. 
Note: though we are processing `run_000001` from the In the Dark set, by default it will be saved as `run_000000` in the output directory.

4. Copy *module_loc_grizzly_surf.yaml* in `vtr_testing/param/scenarios/` and rename it to _my_module_loc_test.yaml_ or something similar.
Open the file in a text editor.

5. Change both *input_data_dir* and *data_dir* to point to the same locations as were used for ModuleVO.
The new run to be processed will be taken from *input_data_dir*.
The processed teach run will be taken from *data_dir* and used to repeat. 
The result will also be stored in *data_dir*.
Change *sim_run* to **'/run_000003'**.
Again, though we are processing `run_000003` from the In the Dark set, by default it will be saved as `run_000001` in the output directory.

6. Optionally, set *save_graph* to true. 
This will add the localization results to the graph as another experience to be used in multi-experience localization.
Note: if you wish to run ModuleLoc again on the same `run_00000X` you should set the *save_graph* to false to avoid repetition of PersistentIds in the graph.

7. In a terminal, run ModuleLocalization by executing the launch file:
   
   ```bash
   roslaunch vtr_testing module_loc_grizzly.launch scenario_params=my_module_loc_test
   ```
If everything is configured correctly, you should see some startup debugging output in the terminal and the same windows as earlier open showing the VO tracks. 
There should also be a window showing the localization results.
If everything is going well, there should be occasional updates in the terminal showing optimization statistics.