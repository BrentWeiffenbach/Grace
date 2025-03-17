#!/bin/bash

verbose=false
revert_pointcloud2=false

function usage() {
    echo "Usage:"
    echo "-p: Reverts pointcloud2 back to its original state"
    echo "-v: Enable verbose logging"
    echo "-h: Display this help message"
}

while getopts :vhp flag
do
    case "${flag}" in
        p) revert_pointcloud2=true;;
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

point_cloud2_path="/opt/ros/melodic/lib/python2.7/dist-packages/ros_numpy/point_cloud2.py"

if [[ -f "$point_cloud2_path" && -r "$point_cloud2_path" ]]; then
    original_code="def get_xyz_points(cloud_array, remove_nans=True, dtype=float):"
    changed_code="def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float64):"
    grep_options=""
    if [ "$verbose" == "false" ]; then
        grep_options="-q"
    fi
    sed_options="-i"

    if [ "$revert_pointcloud2" == "true" ]; then # Has -p flag
        $verbose && echo "Attempting to revert point_cloud2.py to its original state..."
        # Check if it was disablfed before
        if grep $grep_options "$changed_code" "$point_cloud2_path"; then
            $verbose && echo "Found modified line in point_cloud2.py. Attempting to revert..."

            # Revert the line
            sudo -p "Enter your sudo password to modify the protected point_cloud2.py: " sed $sed_options "s|$changed_code|$original_code|" "$point_cloud2_path"
            # Check if line was successfully reverted
            if grep $grep_options "$original_code" "$point_cloud2_path"; then
                # Line was reverted
                $verbose && echo "Line successfully reverted!"
            else
                # Line is still original
                # Non-verbose error
                echo "Failed to revert point_cloud2.py!"
            fi
        else
            $verbose && echo "point_cloud2.py code is still original. Nothing to change. Continuing..."
        fi
    else # Does not have -p flag. Make code modifications
        $verbose && echo "Attempting to apply patch to point_cloud2.py..."
        if ! grep $grep_options "$changed_code" "$point_cloud2_path"; then
            $verbose && echo "Changed code not found. Attempting to modify point_cloud2.py..."
            # Change the line
            sudo -p "Enter your sudo password to modify the protected point_cloud2.py: " sed $sed_options "s|$original_code|$changed_code|" "$point_cloud2_path"
            # Check if line was successfully added
            if grep $grep_options "$changed_code" "$point_cloud2_path"; then
                # Line was successfully changed
                $verbose && echo "point_cloud2 successfully modified!"
            else
                # Line was not found
                # Non-verbose error
                echo "Failed to modify point_cloud2.py!"
            fi
        else
            $verbose && echo "point_cloud2.py code is already modified. Nothing to change. Continuing..."
        fi
    fi
else
    echo "Could not find point_cloud2.py. Please sudo apt-get install ros-melodic-ros-numpy before running the install script."
    exit 1
fi

cd "$grace_dir" || exit
exit 0