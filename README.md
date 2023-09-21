# Visual Teach &amp; Repeat 3 (VT&amp;R3)

## Note

We are still in the process of cleaning up the codebase and having all features of VT&amp;R3 on the main branch.
The current Dockerfile requires GPU support and a CUDA driver capable of supporting 11.7.
Work is in progress to re-enable GPU-free functionality.

- [Main branch](https://github.com/utiasASRL/vtr3) has support for lidar/radar teach and repeat.
- [v3.0.0 tag](https://github.com/utiasASRL/vtr3/tree/v3.0.0) has support for stereo camera teach and repeat (to be merged in the future).

## What is VT&amp;R3?

VT&amp;R3 is a C++ implementation of the Teach and Repeat navigation framework. It enables a robot to be taught a network of traversable paths and then closely repeat any part of network. VT&amp;R3 is designed for easy adaptation to various sensor (camera, lidar, radar, GPS, etc) and robot combinations. So far, we have explored using VT&amp;R3 to perform teach and repeat navigation using a stereo camera, a lidar, or a combination of stereo camera and GPS.

## Reproducing Results of VT&amp;R3 Papers

VT&amp;R3 related papers usually focus on demonstrating one specific feature of VT&amp;R3 instead of the whole system and require additional scripts to run experiements and evaluate results. Therefore, from now on, we will create a separate repository for each paper with instructions on how to reproduce the results.

- [Burnett et al., Are We Ready for Radar to Replace Lidar in All-Weather Mapping and Localization?, IROS'22](https://github.com/utiasASRL/radar_topometric_localization)

## Knowing the Codebase

The following articles will help you get familiar with VT&amp;R3:

- [Installation](https://github.com/utiasASRL/vtr3/wiki/Installation)
- [Getting Started](https://github.com/utiasASRL/vtr3/wiki/Getting-Started)
- [Demo Datasets](https://github.com/utiasASRL/vtr3/wiki/VTR3-Sample-Datasets)

More information can be found in the [wiki page](https://github.com/utiasASRL/vtr3/wiki).

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
