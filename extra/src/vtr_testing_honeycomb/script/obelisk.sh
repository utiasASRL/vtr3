#!/bin/bash

## Define the following environment variables VTH=VTR Testing honeycomb
export VTHROOT=${VTRSRC}/extra/src/vtr_testing_honeycomb
export VTHCONFIG=${VTHROOT}/config
export VTHDATA=${VTRDATA}
export VTHRESULT=${VTRTEMP}/testing/lidar/honeycomb.exp
mkdir -p ${VTHRESULT}

DATASET=${VTHDATA}/utias_20211101_parkinglot_shorter_sequence
ODO_INPUT=rosbag2_2021_11_06-11_41_33
LOC_INPUTS=(
  ## parking lot sequence
  rosbag2_2021_11_01-18_05_58
  rosbag2_2021_11_01-18_10_03
  rosbag2_2021_11_01-18_14_04
  rosbag2_2021_11_01-18_18_34
  # rosbag2_2021_11_06-11_41_33
  rosbag2_2021_11_06-11_45_39
  rosbag2_2021_11_06-11_58_53
  rosbag2_2021_11_06-12_07_03
  rosbag2_2021_11_06-19_57_56
  rosbag2_2021_11_06-20_05_35
  ## short straight line
  # rosbag2_2021_11_06-20_13_42
  # rosbag2_2021_11_06-20_29_52
  # rosbag2_2021_11_06-20_34_11
  # rosbag2_2021_11_06-20_38_08
  # rosbag2_2021_11_06-20_41_24
  # rosbag2_2021_11_06-20_48_00  # This is the one without boreas car
)

## source
echo "[COMMAND] source ${VTRSRC}/extra/install/setup.bash"
source ${VTRSRC}/extra/install/setup.bash

## Odometry
echo "[COMMAND] creating odometry data directory at ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb"
mkdir -p ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}

echo "[COMMAND] running odometry with input ${DATASET}/${ODO_INPUT}"
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_odometry_direct  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
  -p odo_dir:=${DATASET}/${ODO_INPUT}

# backup odometry results
cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
  ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb_v1

# Localization
for LOC_INPUT in "${LOC_INPUTS[@]}"; do
  echo "[COMMAND] copy odometry results to data directory at ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}/honeycomb"
  mkdir -p ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
  cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
    ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}

  echo "[COMMAND] running odometry with input ${DATASET}/${LOC_INPUT}"
  ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_localization_direct \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}/honeycomb \
  -p odo_dir:=${DATASET}/${ODO_INPUT} \
  -p loc_dir:=${DATASET}/${LOC_INPUT}
done