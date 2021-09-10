# Visual Teach &amp; Repeat 3 (VT&amp;R3)

## What is VT&amp;R3?

VT&amp;R3 is a C++ implementation of the Teach and Repeat navigation framework. It enables a robot to be taught a network of traversable paths and then accurately repeat any network portion. VT&amp;R3 is designed for easy adaptation to various sensor (camera/LiDAR/RaDAR/GPS) and robot combinations. The current implementation includes a feature-based visual odometry and localization pipeline that estimates the robot's motion from stereo camera images and a point-cloud-based odometry and localization pipeline for LiDAR sensors.

## Installation and getting started.

The following articles will help you get started with VT&amp;R3:

- [Installation Guide](./wiki/installation_guide.md)
- [Launching VT&amp;R3](./wiki/launch_vtr.md)
- [Adding New Sensors and Robots](./wiki/new_sensor_robot.md)
- [VT&amp;R3 Sample Datasets](./wiki/datasets.md)

More detailed information can be found in the [wiki pages](./README.md) (TODO: create when repo becomes public).

## Citation

Please cite the [following paper](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.20342) when using VT&amp;R3 for your research:

```bibtex
@article{paul2010vtr,
  author = {Furgale, Paul and Barfoot, Timothy D.},
  title = {Visual teach and repeat for long-range rover autonomy},
  journal = {Journal of Field Robotics},
  year = {2010},
  doi = {https://doi.org/10.1002/rob.20342}
}
```

### Additional Citations

- [Multi-Experience Localization](https://ieeexplore.ieee.org/abstract/document/7759303)
  ```bibtex
  @inproceedings{michael2016mel,
    author = {Paton, Michael and MacTavish, Kirk and Warren, Michael and Barfoot, Timothy D.},
    booktitle = {2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    title = {Bridging the appearance gap: Multi-experience localization for long-term visual teach and repeat},
    year={2016},
    doi={10.1109/IROS.2016.7759303}
  }
  ```

## [License](./LICENSE)
