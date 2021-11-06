#!/bin/bash

## Define the following environment variables VTH=VTR Testing honeycomb
export VTHROOT=${VTRSRC}/extra/src/vtr_testing_honeycomb
export VTHCONFIG=${VTHROOT}/config
export VTHDATA=${VTRDATA}/utias_20211101_parkinglot_shorter_sequence
export VTHRESULT=${VTRTEMP}/testing/lidar/honeycomb.exp
mkdir -p ${VTHRESULT}


ODO_INPUT='rosbag2_2021_11_01-18_05_58'
LOC_INPUTS=(
  'rosbag2_2021_11_01-18_10_03'
  'rosbag2_2021_11_01-18_14_04'
  'rosbag2_2021_11_01-18_18_34'
)

## source
echo "[COMMAND] source ${VTRSRC}/extra/install/setup.bash"
source ${VTRSRC}/extra/install/setup.bash

## Odometry
echo "[COMMAND] creating odometry data directory at ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb"
mkdir -p ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}

echo "[COMMAND] running odometry with input ${VTHDATA}/${ODO_INPUT}"
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_odometry_direct  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
  -p odo_dir:=${VTHDATA}/${ODO_INPUT}

# backup odometry results
cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
  ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb_v1

# Localization
for LOC_INPUT in "${LOC_INPUTS[@]}"; do
  echo "[COMMAND] copy odometry results to data directory at ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}/honeycomb"
  mkdir -p ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}
  cp -r ${VTHRESULT}/${ODO_INPUT}/${ODO_INPUT}/honeycomb \
    ${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}

  echo "[COMMAND] running odometry with input ${VTHDATA}/${LOC_INPUT}"
  ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_localization_direct \
  --ros-args  -r __ns:=/vtr  --params-file ${VTHCONFIG}/honeycomb.yaml \
  -p data_dir:=${VTHRESULT}/${ODO_INPUT}/${LOC_INPUT}/honeycomb \
  -p odo_dir:=${VTHDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTHDATA}/${LOC_INPUT}
done