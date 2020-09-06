# steam

STEAM (Simultaneous Trajectory Estimation and Mapping) Engine is an optimization library aimed at solving batch nonlinear optimization problems involving both SO(3)/SE(3) and continuous-time components. This is accomplished by using an iterative Gauss-Newton-style estimator in combination with techniques developed and used by ASRL. With respect to SO(3) and SE(3) components, we make use of the constraint sensitive perturbation schemes discussed in Barfoot and Furgale [1]. STEAM Engine is by no means intended to be the fastest car on the track; the intent is simply to be fast enough for the types of problems we wish to solve, while being both readable and easy to use by people with a basic background in robotic state estimation.

Read the [INSTALL.md](INSTALL.md) file for instruction on how to compile.

[1] Barfoot, T. D. and Furgale, P. T., “_Associating Uncertainty with Three-Dimensional Poses for use in Estimation Problems_,” IEEE Transactions on Robotics, 2014.
