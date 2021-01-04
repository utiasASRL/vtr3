# Offline Tool Tutorial

This tutorial presents a step-by-step guide to running Visual Odometry (VO) and/or Localization using the offline tools. This will help verify that your VTR installation is working correctly and provide a first introduction to using VTR. The examples in this tutorial are not the full VTR system, but simpler binaries that we use to test new parameters or configurations offline on datasets that we already have.

## Table of Contents

- [Offline Tool Tutorial](#offline-tool-tutorial)
  - [Table of Contents](#table-of-contents)
  - [Download and examine an example dataset](#download-and-examine-an-example-dataset)
  - [Module VO](#module-vo)
  - [Module Loc](#module-loc)

## Download and examine an example dataset

Download some data from [Google Drive](https://drive.google.com/drive/folders/1mPgBBOGbbJ6zS2oaua_9PlK7r7nP_N1I?usp=sharing).
This folder contains both new manually collected data (e.g. Nov4 dataset) and converted runs from the 2016 In the Dark experiment with VTR2.

In each dataset, you will see `front_xb3` and `calibration` folders containing ROSbag2 files with the full stereo image stream recorded on the robot (Grizzly) for a _run_, which is a contiguous set of data where the vehicle traversed some trajectory from point A to B.

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

This offline tool allows you to simulate a full Repeat run with visual localization back to a Teach run.
A dataset containing stereo imagery for multiple traversals of the same path is required.

To start, modify both `vtr_testing/config/module_vo.yaml` and `vtr_testing/config/module_loc.yaml` such that their `input_data_dir` parameters point to different runs (ROSbag2 files) from the same dataset while their `results_dir` parameters point to the same folder.
Other parameters such as `start_index` and `stop_index` may also be modified if desired.

Run Module VO using the directions above to produce a Teach run.
Then run Module Loc to simulate a Repeat run: 

```bash
ros2 launch vtr_testing module_loc.launch.py scenario_params:=module_loc.yaml
```

When the end of the run is reached, the window will close and ModuleLoc will exit. You need to `Ctrl-C` complete the process.

Examine the output:

```bash
ros2 run vtr_testing module_loc.py
```

Note: Running Module Loc again in the same results folder with a different input run will demonstrate multi-experience localization (MEL).
Running Module VO again will overwrite the existing Teach run in the `results_dir`.