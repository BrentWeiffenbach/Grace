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

# Check if venv exists
if  [[ ! -d yolovenv && ! -d "../yolovenv" ]]; then
    echo "yolovenv not found. Installing..."

    # Note: the line below is UNTESTED, so if it doesn't work, just run the command that is in the README...
    python3.8 -m virtualenv -p python3.8 yolovenv
    echo "yolovenv created."
fi

#####################
# Change the shebangs
#####################
# Get the full python path for the shebang here
grace_dir="$(cd "$(dirname "$0")" && pwd)"
# Make the script run the same in install or in just Grace
if [[ "$grace_dir" == */install ]]; then
    grace_dir="$(dirname "$grace_dir")"
fi
cd "$grace_dir" || exit

python_path="$grace_dir/yolovenv/bin/python"

if [[ ! -f "$python_path" ]]; then
    echo "Could not find python path."
    exit 1
fi

$verbose && echo "Changing shebangs..."

# Declare files that use relative shebangs here:
python_files=("yolo_detect.py" "bcolors.py" "ultralytics_patch.py" "grace_node.py" "grace_navigation.py" "frontier_search.py")

for python_file in "${python_files[@]}"; do
    full_path="scripts/$python_file"
    if [[ -f "$full_path" ]]; then
        
        # Check first line of the file
        first_line=$(head -n 1 "$full_path")

        # Ensure the first line starts with a shebang
        if [[ "$first_line" =~ ^#! ]]; then
            $verbose && echo "Changing shebang in $full_path..."

            # Change the first line to be the path
            # 1s is find the first line
            # ^#!.*$ is a regex for the entire line if it starts with #!
            # #!$python_path is the shebang to replace it with
            sed -i "1s|^#!.*$|#!$python_path|" "$full_path"
            $verbose && echo "Changed shebang in $full_path."
        else
            $verbose && echo "$full_path does not start with a shebang. Skipping..."
        fi
    else
        $verbose && echo "$full_path does not exist. Skipping..."
    fi
done

$verbose && echo "Done changing shebangs."

##############################
# Change other hardcoded paths
##############################

# Images in yaml
yaml_paths=("my_map" "tutorial") # Must end in yaml

$verbose && echo "Changing yaml file hardcoded paths..."
for yaml_file in "${yaml_paths[@]}"; do
    full_path="maps/$yaml_file.yaml"
    
    if [[ -f $full_path ]]; then
        sed -i "s|image: .*|image: $grace_dir/maps/$yaml_file.pgm|" "$full_path"
    else
        $verbose && echo "$full_path not found. Skipping..."
    fi
done

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
[[ -d out/SLAM ]] || mkdir out/SLAM

# $verbose && echo "Running install_frontier.sh..."
# $verbose && ./install/install_frontier.sh -v || ./install/install_frontier.sh

exit 0 # Successfully exit