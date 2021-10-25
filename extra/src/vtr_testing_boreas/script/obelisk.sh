#!/bin/bash

## Define the following environment variables VTB=VTR Testing Boreas
export VTBROOT=${VTRSRC}/extra/src/vtr_testing_boreas
export VTBCONFIG=${VTBROOT}/config
export VTBDATA=${VTRDATA}/boreas/sequences
export VTBRESULT=${VTRTEMP}/testing/lidar/boreas.exp
mkdir -p ${VTBRESULT}


ODO_INPUT='boreas-2020-12-01-13-26'
LOC_INPUTS=(
  'boreas-2020-12-18-13-44'
  'boreas-2021-01-15-12-17'
  'boreas-2021-03-02-13-38'
  'boreas-2021-04-15-18-55'
  'boreas-2021-04-29-15-55'
  'boreas-2021-06-03-16-00'
  'boreas-2021-07-20-17-33'
  'boreas-2021-09-07-09-35'
)

## source
echo "[COMMAND] source ${VTRSRC}/extra/install/setup.bash"
source ${VTRSRC}/extra/install/setup.bash

## Odometry
echo "[COMMAND] creating odometry data directory at ${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas"
mkdir -p ${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}

echo "[COMMAND] running odometry with input ${VTBDATA}/${ODO_INPUT}"
ros2 run vtr_testing_boreas vtr_testing_boreas_odometry_direct  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml \
  -p data_dir:=${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas \
  -p odo_dir:=${VTBDATA}/${ODO_INPUT}

# backup odometry results
cp -r ${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas \
  ${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas_v1

# Localization
for LOC_INPUT in "${LOC_INPUTS[@]}"; do
  echo "[COMMAND] copy odometry results to data directory at ${VTBRESULT}/${ODO_INPUT}/${LOC_INPUT}/boreas"
  mkdir -p ${VTBRESULT}/${ODO_INPUT}/${LOC_INPUT}
  cp -r ${VTBRESULT}/${ODO_INPUT}/${ODO_INPUT}/boreas \
    ${VTBRESULT}/${ODO_INPUT}/${LOC_INPUT}

  echo "[COMMAND] running odometry with input ${VTBDATA}/${LOC_INPUT}"
  ros2 run vtr_testing_boreas vtr_testing_boreas_localization_direct \
  --ros-args  -r __ns:=/vtr  --params-file ${VTBCONFIG}/boreas.yaml \
  -p data_dir:=${VTBRESULT}/${ODO_INPUT}/${LOC_INPUT}/boreas \
  -p odo_dir:=${VTBDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTBDATA}/${LOC_INPUT}
done