#!/bin/bash

if [ $# -lt 2 ]; then
    echo "Run as:"
    echo "./urdf_from_xacro.sh path-to-xacro path-to-urdf"
    exit 1
fi

XACRO=$1
URDF=$2

ros2 run xacro xacro $1 > $2
