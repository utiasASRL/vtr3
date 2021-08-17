#!/bin/bash

#The number of old bags to keep.:
NUM_OLD_BAGS=1
#The base directory name:
DIR_NAME="${ASRL_DATA_DIR}/Diagnostics"
#The sub directory name:
SUB_NAME="grizzly_log"
#The date stamp:
DATE_STAMP=$(date -u +"%Y_%m_%dT%H_%M_%S%N%Z")
#The new folder:
NEW_FOLDER="${DIR_NAME}"/"${SUB_NAME}_${DATE_STAMP}"
#The split time (in minutes):
SPLIT_TIME=1
#The bagging script to run:
BAG_CMD="rosbag record --split --duration=${SPLIT_TIME}m -o ${NEW_FOLDER}/auto"
#The topics to bag:
BAG_TOPICS="
/cmd_drive
/cmd_vel
/diagnostics
/diagnostics_agg
/diagnostics_toplevel_state
/estop
/imu/temperature
/joy
/mcu/ambience
/mcu/body_temp
/mcu/energy
/mcu/estop
/mcu/fan
/mcu/status
/motors/encoders
/motors/front_left/cmd
/motors/front_left/feedback
/motors/front_left/status
/motors/front_right/cmd
/motors/front_right/feedback
/motors/front_right/status
/motors/rear_left/cmd
/motors/rear_left/feedback
/motors/rear_left/status
/motors/rear_right/cmd
/motors/rear_right/feedback
/motors/rear_right/status
/rosout
/safe_cmd_drive
"

#Make the new subdirectory:
mkdir -p "${NEW_FOLDER}"

#Remove the old bag files
#Get the list of folders ordered :
DIR_LISTING=$(ls -t "${DIR_NAME}")

#Iterate through the list, counting as we go. When the counter is greater than N, start deleting!
#Start with 0 files, as we just created one we don't want to delete.
FILE_NUM=0

#Iterate over each one
for FILE in $DIR_LISTING; do
  #Check if this is the N+1-th or greater file:
  if [ "$FILE_NUM" -gt "${NUM_OLD_BAGS}" ]; then
    #It is! Remove:
    echo "rm -r ${DIR_NAME}/${FILE}"
    rm -r "${DIR_NAME}/${FILE}"
  fi
  #It is less than or equal to N, don't delete

  #Increment counter
  let FILE_NUM=FILE_NUM+1
done
#All done cleaning up.


#Start bagging:

#Run rosbag
#echo "${BAG_CMD} ${BAG_TOPICS}"
${BAG_CMD} ${BAG_TOPICS}
