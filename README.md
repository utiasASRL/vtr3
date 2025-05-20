# Visual Teach &amp; Repeat 3 (VT&amp;R3)

## What is VT&amp;R3?

VT&amp;R3 is a C++ implementation of the Teach and Repeat navigation framework. It enables a robot to be taught a network of traversable paths and then closely repeat any part of the network. VT&amp;R3 is designed for easy adaptation to various sensors (camera, lidar, radar, GPS, etc) and robot combinations. So far, we have explored using VT&amp;R3 to perform teach and repeat navigation using a stereo camera, a lidar, or a combination of a stereo camera and GPS.


## Software Support
This repository contains active support for the following feature
* Multi-experience localization with a Stereo Camera
* Deep-learned visual features with a Stereo Camera
* LiDAR ICP odometry and localization

With support for so many sensors, the repository has grown quite large. 
To reduce the required compilation time an environment variable `VTR_PIPELINE` has been added to allow for the pipeline to be selected at compile time, instead of run time. 
The supported pipelines are:
* `LIDAR`
* `VISION`
* `RADAR`
* `RADAR-LIDAR`
  
If the variable is unset, then all pipelines will be compiled, and the user can select at run time through the config file parameter `pipeline.type` which pipeline to use.

The primary support version of VTR requires an NVidia Driver with Cuda capabilities.
The current Dockerfile requires a CUDA driver capable of supporting 11.7.
A GPU is required for all versions of the vision (camera) pipeline and features for LiDAR and RADAR that use PyTorch models for processing.

If no GPU is available, a CPU only version is available, but **only for LiDAR**.
Note that the CPU version of TorchLib is installed for easier compilation but the models are unlikely to run fast enough on a CPU to be useful. 
If you have no NVidia drivers, then you will need to add an empty file called `COLCON_IGNORE` in the folder `main/src/deps/gpusurf`. This will skip this compilation. 

## Reproducing Results of VT&amp;R3 Papers

VT&amp;R3 related papers usually focus on demonstrating one specific feature of VT&amp;R3 instead of the whole system and require additional scripts to run experiments and evaluate results. Therefore, from now on, we will create a separate repository for each paper with instructions on how to reproduce the results.

- [Burnett et al., Are We Ready for Radar to Replace Lidar in All-Weather Mapping and Localization?, IROS'22](https://github.com/utiasASRL/radar_topometric_localization)

## Knowing the Codebase

The following articles will help you get familiar with VT&amp;R3:

- [Installation](https://github.com/utiasASRL/vtr3/wiki/Installation)
- [Getting Started](https://github.com/utiasASRL/vtr3/wiki/Getting-Started)
- [Demo Datasets](https://github.com/utiasASRL/vtr3/wiki/VTR3-Sample-Datasets)

More information can be found on the [wiki page](https://github.com/utiasASRL/vtr3/wiki).

## Citation

Please cite the following [paper](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.20342) when using VT&amp;R3 for your research:

```bibtex
@article{paul2010vtr,
  author = {Furgale, Paul and Barfoot, Timothy D.},
  title = {Visual teach and repeat for long-range rover autonomy},
  journal = {Journal of Field Robotics},
  year = {2010},
  doi = {https://doi.org/10.1002/rob.20342}
}
```

## [License](./LICENSE)
