#!/bin/bash
verbose=false
no_build=false

function usage() {
    echo "Usage:"
    echo "-v: Enable verbose logging"
    echo "-b: Disable building"
    echo "-h: Display this help message"
}

while getopts :vhb flag
do
    case "${flag}" in
        v) verbose=true;;
        h) usage;;
        b) no_build=true;;
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
sudo apt-get install -qq ros-melodic-pybind11-catkin ros-melodic-eigen-stl-containers ros-melodic-random-numbers ros-melodic-shape-msgs ros-melodic-object-recognition-msgs ros-melodic-graph-msgs ros-melodic-ruckig ros-melodic-eigenpy ros-melodic-rosparam-shortcuts
sudo apt-get install -qq libconsole-bridge-dev
sudo apt-get install -qq libccd-dev libfcl-dev libglew-dev
sudo apt-get install -qq ros-melodic-pcl-ros

python -m pip install pyassimp==3.3

$verbose && echo "Cloning required repositories..."
[ -d "rviz_visual_tools" ] || git clone -b master https://github.com/PickNikRobotics/rviz_visual_tools
[ -d "geometric_shapes" ] || git clone -b noetic-devel https://github.com/ros-planning/geometric_shapes.git geometric_shapes
[ -d "srdfdom" ] || git clone -b noetic-devel https://github.com/moveit/srdfdom
[ -d "moveit" ] || git clone -b main https://github.com/BrentWeiffenbach/moveit_planar_modification.git moveit
[ -d "moveit_msgs" ] || git clone -b master https://github.com/moveit/moveit_msgs.git moveit_msgs
[ -d "moveit_resources" ] || git clone -b master https://github.com/moveit/moveit_resources.git moveit_resources
[ -d "moveit_tutorials" ] || git clone -b master https://github.com/moveit/moveit_tutorials.git moveit_tutorials
[ -d "moveit_visual_tools" ] || git clone -b master https://github.com/moveit/moveit_visual_tools.git moveit_visual_tools
$verbose && echo "Cloned github repositories!"

cd ~ || exit
[ -d "ompl-1.6.0" ] || curl -sLf https://ompl.kavrakilab.org/install-ompl-ubuntu.sh | bash

cd "$grace_dir" || exit
cd ../.. # Go to catkin_ws

$no_build || catkin clean
$no_build || catkin build moveit
$no_build || source devel/setup.bash

cd "$grace_dir" || exit

exit 0