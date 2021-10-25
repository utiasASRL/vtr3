## Define the following environment variables VTB=VTR Testing Boreas
export VTBROOT=${VTRSRC}/extra/src/vtr_testing_boreas
export VTBCONFIG=${VTBROOT}/config
export VTBDATA=${VTRDATA}/boreas/sequences
export VTBRESULT=${VTRTEMP}/testing/lidar/boreas.exp
mkdir -p ${VTBRESULT}

## Perform odometry on a sequence directly
ODO_INPUT=boreas-2020-12-01-13-26
#        package            executable                                      namespace      parameter file                          data directory (output dir)       input directory
ros2 run vtr_testing_boreas vtr_testing_boreas_odometry_direct  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml \
  -p data_dir:=${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas \
  -p odo_dir:=${VTBDATA}/${ODO_INPUT}

## Perform localization on a sequence directly
ODO_INPUT=boreas-2020-12-01-13-26
LOC_INPUT=boreas-2020-12-18-13-44
ros2 run vtr_testing_boreas vtr_testing_boreas_localization_direct \
  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml \
  -p data_dir:=${VTBRESULT}/${ODO_INPUT}/${LOC_INPUT}/boreas \
  -p odo_dir:=${VTBDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTBDATA}/${LOC_INPUT}

## Perform offline tasks
ODO_INPUT=boreas-2020-12-01-13-26
LOC_INPUT=boreas-2020-12-18-13-44
ros2 run vtr_testing_boreas vtr_testing_boreas_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml \
  -p data_dir:=${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas
ros2 run vtr_testing_boreas vtr_testing_boreas_dynamic_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml \
  -p data_dir:=${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas
ros2 run vtr_testing_boreas vtr_testing_boreas_inter_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml \
  -p data_dir:=${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas

## Perform odometry/localization on a sequence using ROS subscriber
ros2 run vtr_testing_boreas vtr_testing_boreas_odometry_ros  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml  -p data_dir:=${VTBRESULT}/boreas
ros2 run vtr_testing_boreas vtr_testing_boreas_localization_ros  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml  -p data_dir:=${VTBRESULT}/boreas
