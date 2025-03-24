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

grace_dir="$(cd "$(dirname "$0")" && pwd)"
# Make the script run the same in install or in just Grace
if [[ "$grace_dir" == */install ]]; then
    grace_dir="$(dirname "$grace_dir")"
fi
cd "$grace_dir" || exit

# Check if venv exists
if  [[ ! -d yolovenv && ! -d "../yolovenv" ]]; then
    echo "yolovenv not found. Installing..."

    # Note: the line below is UNTESTED, so if it doesn't work, just run the command that is in the README...
    python3.8 -m pip install virtualenv
    python3.8 -m virtualenv -p python3.8 yolovenv
    echo "yolovenv created."
fi

# python_files=("yolo_detect.py" "grace_node.py" "grace_navigation_node.py" "frontier_search.py")

#################################
# Disable verbose logging in YOLO
#################################
# Use argument -y to enable YOLO verbose logging
$verbose && echo "Running disable_yolo_spam.sh..."
# shellcheck disable=SC2048
# shellcheck disable=SC2086
./install/disable_yolo_spam.sh $*

$verbose && echo "Running install_aws_robotmaker.sh..."
# shellcheck disable=SC2048
# shellcheck disable=SC2086
./install/install_aws_robomaker.sh $*

$verbose && echo "Running change_pointcloud.sh..."
# shellcheck disable=SC2048
# shellcheck disable=SC2086
./install/change_pointcloud.sh $*

# TODO: Add verbosity to this
[[ -d out ]] || mkdir out
[[ -d out/SLAM ]] || mkdir out/SLAM

# $verbose && echo "Running install_frontier.sh..."
# $verbose && ./install/install_frontier.sh -v || ./install/install_frontier.sh

exit 0 # Successfully exit