## Define the following environment variables VTB=VTR Testing Boreas
export VTBROOT=${VTRSRC}/extra/src/vtr_testing_boreas
export VTBCONFIG=${VTBROOT}/config
export VTBDATA=${VTRDATA}/boreas/sequences
export VTBRESULT=${VTRTEMP}/testing/lidar/boreas.exp

## Perform odometry on a sequence directly
#        package            executable                                      namespace      parameter file                          data directory (output dir)       input directory
ros2 run vtr_testing_boreas vtr_testing_boreas_odometry_direct  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml  -p data_dir:=${VTBRESULT}/boreas  -p input_dir:=${VTBDATA}/boreas-2020-12-01-13-26

## Perform localization on a sequence directly
ros2 run vtr_testing_boreas vtr_testing_boreas_localization_direct  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml  -p data_dir:=${VTBRESULT}/boreas  -p input_dir:=${VTBDATA}/boreas-2021-03-02-13-38


## Perform odometry/localization on a sequence using ROS subscriber
ros2 run vtr_testing_boreas vtr_testing_boreas_odometry_ros  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml  -p data_dir:=${VTBRESULT}/boreas
ros2 run vtr_testing_boreas vtr_testing_boreas_localization_ros  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml  -p data_dir:=${VTBRESULT}/boreas

## Perform offline tasks
ros2 run vtr_testing_boreas vtr_testing_boreas_dynamic_detection  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml  -p data_dir:=${VTBRESULT}/boreas
ros2 run vtr_testing_boreas vtr_testing_boreas_intra_exp_merging  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml  -p data_dir:=${VTBRESULT}/boreas