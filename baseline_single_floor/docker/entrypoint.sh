#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [ -f /workspace/SimEnv/devel/setup.bash ]; then
    source /workspace/SimEnv/devel/setup.bash
fi

exec "$@"
