#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
source "/opt/ropod/ros/setup.bash"
exec "$@"
