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

# Go to catkin_ws/src
cd ..
if [[ ! -d aws-robomaker-small-house-world ]]; then
    echo "aws-robomaker-small-house-world. Installing..."
    git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git
    echo "aws-robomaker-small-house-world installed."

        # Ask user if they want to build with catkin build
    echo "Build aws-robomaker-small-house-world using catkin build?"
    read -n 1 -r -p "(y/n): " "build"
    echo -e # Print a new line
    if [ "$build" != "y" ]; then
        echo "Please build aws-robomaker-small-house-world and run the install script again."
        exit 0
    fi

    # User wants to build using catkin build
    # Install catkin build
    $verbose && echo "Installing dependencies for catkin build..."
    sudo -p "Enter sudo password to install catkin: " apt install python3-catkin-tools
    sudo apt-get install ros-melodic-roslint
    $verbose && echo "Installed dependencies for catkin build"
    cd .. # go to catkin_ws
    
    $verbose && echo "Building aws-robomaker-small-house-world..."
    catkin build aws-robomaker-small-house-world
    verbose && echo "Built aws-robomaker-small-house-world!"
fi

cd "$grace_dir" || exit

exit 0