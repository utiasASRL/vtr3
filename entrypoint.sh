#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

# Fetch host UID, GID, and Username from env variables, or default to 1000/asrl
USER_ID=${USER_ID:-1000}
GROUP_ID=${GROUP_ID:-1000}
USER_NAME=${USER_NAME:-asrl}

if [ "$(id -u)" -eq 0 ]; then
    echo "Setting up container user: $USER_NAME ($USER_ID:$GROUP_ID)"

    # 1. Create the group if it doesn't exist
    if ! getent group "$USER_NAME" >/dev/null; then
        groupadd -g "$GROUP_ID" "$USER_NAME"
    fi

    # 2. Create the user if they don't exist
    if ! id -u "$USER_NAME" >/dev/null 2>&1; then
        useradd -u "$USER_ID" -g "$GROUP_ID" -m -s /bin/bash "$USER_NAME"
    else
        mkdir -p "/home/$USER_NAME"
    fi

    # 3. Fix home ownership (not recursive: don't walk the bind-mounted vtr3 tree)
    chown "$USER_ID:$GROUP_ID" "/home/$USER_NAME"
    export HOME="/home/$USER_NAME"
else
    # Already non-root (e.g. plain `docker exec`): skip user setup and gosu
    USER_NAME=$(id -un)
    export HOME=${HOME:-/home/$USER_NAME}
fi

export VTRROOT=${HOME}/ASRL/vtr3
export VTRSRC=${VTRROOT}/src
export VTRDATA=${VTRROOT}/data
export VTRTEMP=${VTRROOT}/temp
export VTRMODELS=${VTRROOT}/models
export GRIZZLY=${VTRROOT}/grizzly
export WARTHOG=${VTRROOT}/warthog
export HUNTER=${VTRROOT}/hunter
export VTRUI=${VTRSRC}/main/src/vtr_gui/vtr_gui/vtr-gui

# 4. Source ROS + the VTR workspace
#    set +e because ROS setup scripts can return non-zero on harmless warnings
set +e
. /opt/ros/humble/setup.bash
if [ -f "$VTRSRC/main/install/setup.bash" ]; then
    . "$VTRSRC/main/install/setup.bash"
else
    echo "Note: $VTRSRC/main/install/setup.bash not found (workspace not built yet?)"
fi
set -e

# 5. Start in VTRROOT (working directory is inherited through exec)
cd "$VTRROOT" || echo "Warning: $VTRROOT not found (is the volume mounted?)"

# 6. Run the command, stepping down from root if needed
if [ "$(id -u)" -eq 0 ]; then
    exec gosu "$USER_NAME" "$@"
else
    exec "$@"
fi