#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
source "/ropod_com_mediator_ws/devel/setup.bash"

# Launch the com_mediator
roslaunch ropod_com_mediator com_mediator.launch debug_mode:=true
exec "$@"
