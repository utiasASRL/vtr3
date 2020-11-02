# Offline Tool Tutorial

This tutorial presents a step-by-step guide to running Visual Odometry (VO) and/or Localization using the offline tools. This will help verify that your VTR installation is working correctly and provide a first introduction to using VTR. The examples in this tutorial are not the full VTR system, but simpler binaries that we use to test new parameters or configurations offline on datasets that we already have.

## Table of Contents

- [Offline Tool Tutorial](#offline-tool-tutorial)
  - [Table of Contents](#table-of-contents)
  - [Download and examine an example dataset](#download-and-examine-an-example-dataset)
  - [Module VO](#module-vo)
  - [Module Loc](#module-loc)

## Download and examine an example dataset

Download the dataset from [Google Drive](https://drive.google.com/drive/folders/1zc4E1iJfY9wrEWbWM25qi-Y4TPtxhGuz?usp=sharing).

In each dataset, you will see a number of `run_00000X` folders. Each of these folders contains all the images and GPS data recorded on the robot (grizzly) for a _run_, which is a contiguous set of data where the vehicle traversed some trajectory from point A to B.

By convention, all data associated with a sensor is stored in the `sensorData` folder of a run, including its products (which might be features or landmarks in the case of images), under a subfolder name associated with the sensor name. In this case `front_xb3` refers to the Point Grey XB3 stereo camera mounted on the front of the Grizzly. The data is also stored under a name referring to its type. In this case `processed_stereo_images` implies that the data is a set of stereo images that have had some processing applied.

## Module VO

Module VO scenario specific parameter file: `vtr_testing/config/module_vo.yaml`.

- `input_data_dir`: directory that stores the dataset, defaults to `~/ASRL/dataset/<some dataset>`.
- `results_dir`: directory to store the module vo specific test results.
- `data_dir`: directory to store VT&R outputs, e.g. pose graphs.
- `sim_run`: which run to load (TODO: not used until data is assumed to be collected VTR&3).
- `start_index` & `stop_index`: range of images in dataset to load (`[start_index, stop_index)`).

Module VO launch file: `vtr_testing/launch/module_vo.launch.py`.

- `scenario_params`: which parameter file to use, choose the above file.

Run Module VO:

```bash
ros2 launch vtr_testing module_vo.launch.py scenario_params:=module_vo.yaml
```

When the end of the run is reached, the window will close and ModuleVO will exit. You need to `Ctrl-C` complete the process.

Examine the output:

```bash
ros2 run vtr_testing module_vo.py
```

## Module Loc

TODO.
