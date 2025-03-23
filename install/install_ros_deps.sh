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

python -m pip install netifaces # Install it globally

cd .. # Go to catkin_ws/src
[ -d "turtlebot_interactions" ] || git clone https://github.com/turtlebot/turtlebot_interactions.git
cd .. # Go to catkin_ws

if [ ! -d "src/turtlebot" ]; then
    $verbose && echo "Running turtlebot install script..."
    curl -sLf https://raw.githubusercontent.com/gaunthan/Turtlebot2-On-Melodic/master/install_basic.sh | bash
    $verbose && echo "Finished turlebot install script!"
fi

sudo apt-get install -qq ros-melodic-gmapping ros-melodic-move-base ros-melodic-dwa-local-planner ros-melodic-rosserial ros-melodic-rosserial-arduino

catkin clean
catkin build
source devel/setup.bash
cd "$grace_dir" || exit 


exit 0