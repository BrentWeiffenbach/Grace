#!/bin/bash
# These are some commands I had to run to install the first time. Uncomment them if there are issues with the script to see if it fixes things.
# sudo apt-get update
# sudo apt install rosbash
# sudo apt-get install ros-melodic-gazebo* ros-melodic-robot-state-publisher ros-melodic-xacro
# sudo apt install python3.8.0

verbose=false

function usage() {
    echo "Usage:"
    echo "-v: Enable verbose logging"
    echo "-p: Reverts point_cloud2 back to its original state"
    echo "-y: Enable yolo logging to console (not recommended)"
    echo "-h: Display this help message"
}

while getopts :vhyp flag
do
    case "${flag}" in
        v) verbose=true;;
        h) usage;;
        y) ;;
        p) ;;
        *) ;;
    esac
done

./grace_navigation/install/install.sh $*

exit 0 # Successfully exit