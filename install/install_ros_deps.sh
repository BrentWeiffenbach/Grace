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

cd ../.. # Go to catkin_ws
curl -sLf https://raw.githubusercontent.com/gaunthan/Turtlebot2-On-Melodic/master/install_basic.sh | bash
sudo apt-get install ros-melodic-gmapping ros-melodic-move-base ros-melodic-dwa-local-planner
sudo apt-get install ros-melodic-turtlebot-apps
cd src # Go to catkin_ws/src
git clone https://github.com/turtlebot/turtlebot_interactions.git
cd .. # Go to catkin_ws
catkin build
source ~/.bashrc
python -m pip install netifaces
cd "$grace_dir" || exit 


exit 0