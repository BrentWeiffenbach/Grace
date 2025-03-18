#!/bin/bash
verbose=false

function usage() {
    echo "Usage:"
    echo "-v: Enable verbose logging"
    echo "-h: Display this help message"
}

while getopts :vh flag
do
    case "${flag}" in
        v) verbose=true;;
        h) usage;;
        *) ;;
    esac
done

grace_dir="$(cd "$(dirname "$0")" && pwd)"
# Make the script run the same in install or in just Grace
if [[ "$grace_dir" == */install ]]; then
    grace_dir="$(dirname "$grace_dir")"
fi
cd "$grace_dir" || exit

cd .. # Go to catkin_ws/src
git clone -b main https://github.com/BrentWeiffenbach/moveit_planar_modification.git moveit
git clone -b melodic-devel https://github.com/moveit/moveit_msgs.git moveit_msgs
git clone -b melodic-devel https://github.com/moveit/moveit_resources.git moveit_resources
git clone -b melodic-devel https://github.com/moveit/moveit_tutorials.git moveit_tutorials
git clone -b melodic-devel https://github.com/moveit/moveit_visual_tools.git moveit_visual_tools
git clone -b melodic-devel https://github.com/moveit/srdfdom

cd .. # Go to catkin_ws

catkin build

cd "$grace_dir" || exit