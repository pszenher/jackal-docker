#!/bin/bash
set -e

# setup jackal ros environment (for use in Docker containers)
source "/etc/ros/setup.bash" --
exec "$@"
