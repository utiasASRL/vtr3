#!/usr/bin/env bash
SETUP_DIR=$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)

ROS_PACKAGE_PATH="$(dirname "${SETUP_DIR})")":${ROS_PACKAGE_PATH}
export ROS_PACKAGE_PATH
