#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: ./build_package.sh <package_name>"
    exit 1
fi

colcon build --symlink-install --packages-select "$1"