# Set the ASRL Code directory
# TODO not used in VTR3
export ASRL_CODE_DIR=~/asrl-misc-code
# Set the charlottetown root folder, it is assumed that all workspaces (ros_osrf, extras, utias, YOUR_LAB) and the scripts/rosinstalls/logs reside in this folder
# TODO not true in VTR3
export UTIAS_ROS_DIR=~/charlottetown
# Set the name of your lab workspace
export UTIAS_LAB_WS=asrl
# Add the helper scripts to the PATH
export PATH="${UTIAS_ROS_DIR}"/scripts/:$PATH
# Set the data logging directory
export ASRL_DATA_DIR="${UTIAS_ROS_DIR}"/logs

# Source the vtr2 workspace (which includes all parents)
VTRSETUP=${UTIAS_ROS_DIR}/utiasASRL/vtr2/devel/repo/setup.bash
source $VTRSETUP
echo "VT&R setup.bash sourced: ${VTRSETUP}"

# Catkin recursive cmd in src
function catkin_src_cmd () {
    test $# -lt 1 && echo "Must provide a command to run!" && return 1
    test -d src || echo "There is no 'src' subdirectory!"
    test -d src || return 1
    local command="cd {} && echo \"--- In: {} ---\" && $@"
    find ./src -mindepth 1 -maxdepth 1 -type d -exec sh -c "$command" \;
}