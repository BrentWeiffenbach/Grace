#!/bin/bash

verbose=false

while getopts v flag
do
    case "${flag}" in
        v) verbose=true;;
    esac
done

# Go to catkin_ws/src
grace_dir="$(cd "$(dirname "$0")" && pwd)"
cd ..
if [[ ! -d frontier_exploration ]]; then
    echo "frontier_exploration not found. Installing..."
    git clone https://github.com/paulbovbel/frontier_exploration
    echo "frontier_exploration installed."

    # Ask user if they want to build with catkin build
    echo "Build frontier_exploration using catkin build?"
    read -n 1 -p "(y/n): " "build"
    echo -e # Print a new line
    if [ "$build" != "y" ]; then
        echo "Please build frontier_exploration and run the install script again."
        exit 0
    fi

    # User wants to build using catkin build
    # Install catkin build
    $verbose && echo "Installing dependencies for catkin build..."
    sudo apt install python3-catkin-tools
    sudo apt-get install ros-melodic-roslint
    $verbose && echo "Installed dependencies for catkin build"
    cd .. # go to catkin_ws
    
    $verbose && echo "Building frontier_exploration..."
    catkin build frontier_exploration
    verbose && echo "Built frontier_exploration!"
fi
# Go back to grace
cd $grace_dir
exit 0