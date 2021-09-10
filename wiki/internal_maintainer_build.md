## (TODO: Yuchen) Build in debug mode

```bash
cd ${VTRSRC}/main
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"  # Debug build with code coverage test
colcon test --event-handlers console_cohesion+ # Will also run style check for c++, python, cmake and xml files.
colcon test-result  # Summary: xx tests, 0 errors, 0 failures, 0 skipped
lcov --capture --directory build/ --output-file vtr3_coverage_report.info
genhtml vtr3_coverage_report.info --output-directory vtr3_coverage_report
```

Open the html report at `vtr3_coverage_report/index.html` to see code coverage. Note that this option is for development only and not intended to be used for end-to-end online/offline experiments (such as the video demos below).

State estimation in VT&R (odometry, mapping and localization) can be built to run deterministically by adding the following flags to the [common cmake file](./main/src/vtr_common/vtr_include.cmake), given that data arrive at a sufficiently slow rate.

```bash
add_definitions(-DVTR_DETERMINISTIC)  # disable multi-threading in VTR state estimation and force any GPU job to run deterministically
add_definitions(-DSTEAM_DEFAULT_NUM_OPENMP_THREADS=1)  # disable multi-threading in STEAM
```
