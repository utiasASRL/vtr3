## Define the following environment variables VTH=VTR Testing Honeycomb
export VTHROOT=${VTRSRC}/extra/src/vtr_testing_honeycomb
export VTHCONFIG=${VTHROOT}/config
export VTHDATA=${VTRDATA}/utias_2021_parkinglot_rosbag
export VTHRESULT=${VTRTEMP}/testing/lidar/honeycomb.exp
mkdir -p ${VTHRESULT}

## Perform odometry on a sequence directly
DATASET=${VTHDATA}/utias_20211101_parkinglot_shorter_sequence
ODO_INPUT=rosbag2_2021_11_06-11_58_53
#        package            executable                                      namespace      parameter file                          data directory (output dir)       input directory
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_odometry_direct  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
  -p odo_dir:=${DATASET}/${ODO_INPUT}

## Perform localization on a sequence directly (with a specified point map version)
ODO_INPUT=rosbag2_2021_11_06-11_58_53
LOC_INPUT=rosbag2_2021_08_23-11_58_52
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_localization_direct \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p localization.recall.map_version:=point_map_v0 \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}/honeycomb \
  -p odo_dir:=${DATASET}/${ODO_INPUT} \
  -p loc_dir:=${DATASET}/${LOC_INPUT}

## Perform offline tasks
ODO_INPUT=rosbag2_2021_11_01-18_05_58
LOC_INPUT=rosbag2_2021_08_23-11_58_52
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_dynamic_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_inter_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb
