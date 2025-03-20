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
sudo apt-get install ros-melodic-pybind11-catkin ros-melodic-eigen-stl-containers ros-melodic-random-numbers ros-melodic-shape-msgs ros-melodic-object-recognition-msgs ros-melodic-graph-msgs ros-melodic-ruckig
sudo apt-get install libconsole-bridge-dev
sudo apt-get install libccd-dev libfcl-dev libglew-dev
sudo apt-get install ros-melodic-pcl-ros

$verbose && echo "Cloning required repositories..."
# git clone -b master https://github.com/PickNikRobotics/rviz_visual_tools
# git clone -b noetic-devel https://github.com/ros-planning/geometric_shapes.git geometric_shapes
# git clone -b noetic-devel https://github.com/moveit/srdfdom
# git clone -b main https://github.com/BrentWeiffenbach/moveit_planar_modification.git moveit
# git clone -b master https://github.com/moveit/moveit_msgs.git moveit_msgs
# git clone -b master https://github.com/moveit/moveit_resources.git moveit_resources
# git clone -b master https://github.com/moveit/moveit_tutorials.git moveit_tutorials
# git clone -b master https://github.com/moveit/moveit_visual_tools.git moveit_visual_tools
$verbose && echo "Cloned github repositories!"

cd ~ || exit
[ -d "ompl-1.6.0" ] || curl -sLf https://ompl.kavrakilab.org/install-ompl-ubuntu.sh | bash
# [ -d "ompl" ] || git clone --single-branch --branch 1.5.2 https://github.com/ompl/ompl.git
# cd ompl || exit
# mkdir -p build/Release
# cd build/Release || exit
# cmake -DCMAKE_BUILD_TYPE=Release ../..
# make -j$(nproc)
# sudo make install

cd .. # Go to catkin_ws

# catkin clean
# catkin build

cd "$grace_dir" || exit

exit 0