#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

# Fetch host UID, GID, and Username from env variables, or default to 1000/asrl
USER_ID=${USER_ID:-1000}
GROUP_ID=${GROUP_ID:-1000}
USER_NAME=${USER_NAME:-asrl}

echo "Setting up container user: $USER_NAME ($USER_ID:$GROUP_ID)"

# 1. Create the group if it doesn't exist
if ! getent group "$USER_NAME" >/dev/null; then
    groupadd -g "$GROUP_ID" "$USER_NAME"
fi

# 2. Create the user if they don't exist
if ! id -u "$USER_NAME" >/dev/null 2>&1; then
    # -m creates the home directory (/home/username) internally
    useradd -u "$USER_ID" -g "$GROUP_ID" -m -s /bin/bash "$USER_NAME"
else
    # If the user exists but home directory is missing, create it
    mkdir -p "/home/$USER_NAME"
fi

# 3. Ensure the home directory has the correct permissions
chown -R "$USER_ID:$GROUP_ID" "/home/$USER_NAME"

export HOME="/home/$USER_NAME"
export VTRROOT=${HOME}/ASRL/vtr3
export VTRSRC=${VTRROOT}/src \
  VTRDATA=${VTRROOT}/data \
  VTRTEMP=${VTRROOT}/temp \
  VTRMODELS=${VTRROOT}/models \
  GRIZZLY=${VTRROOT}/grizzly \
  WARTHOG=${VTRROOT}/warthog \
  HUNTER=${VTRROOT}/hunter \

export VTRUI=${VTRSRC}/main/src/vtr_gui/vtr_gui/vtr-gui
source /opt/ros/humble/setup.bash

# 4. Step down from root and execute the CMD passed to the container
exec gosu "$USER_NAME" "$@"