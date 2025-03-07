#!/bin/bash

verbose=false
do_verbose_yolo_logging=false

function usage() {
    echo "Usage:"
    echo "-v: Enable verbose logging"
    echo "-y: Enable yolo logging to console (not recommended)"
    echo "-h: Display this help message"
}

while getopts :vyh flag
do
    case "${flag}" in
        v) verbose=true;;
        y) do_verbose_yolo_logging=true;;
        h) usage;;
        *) ;;
    esac
done

# Go to catkin_ws/src
grace_dir="$(cd "$(dirname "$0")" && pwd)"
# Make the script run the same in install or in just Grace
if [[ "$grace_dir" == */install ]]; then
    grace_dir="$(dirname "$grace_dir")"
fi

cd "$grace_dir" || exit


predictor_py_path="$grace_dir/yolovenv/lib/python3.8/site-packages/ultralytics/engine/predictor.py"
if [[ -f "$predictor_py_path" && -r "$predictor_py_path" && -w "$predictor_py_path" ]]; then
    disabled_verbose_code="            self.args.verbose = False"
    grep_options=""
    if [ "$verbose" == "false" ]; then
        grep_options="-q"
    fi
    sed_options="-i"

    if [ "$do_verbose_yolo_logging" == "true" ]; then # Disable YOLO logging (remove supressing line)
        $verbose && echo "Attempting to enable verbose YOLO logging..."
        # Check if it was disablfed before
        if grep $grep_options "$disabled_verbose_code" "$predictor_py_path"; then
            $verbose && echo "Disable verbose line found in predictor.py. Attempting to remove line..."

            # Remove the line
            sed $sed_options "/$disabled_verbose_code/d" "$predictor_py_path"
            # Check if line was successfully removed
            if ! grep $grep_options "$disabled_verbose_code" "$predictor_py_path"; then
                # Line was removed
                $verbose && echo "Line removed!"
            else
                # Line was still found
                # Non-verbose error
                echo "Failed to remove the verbose logging suppressor line in predictor.py!"
            fi
        else
            $verbose && echo "Disable verbose line not found. Nothing to change. Continuing..."
        fi
    else # Disable YOLO logging (add supressing line)
        $verbose && echo "Attempting to disable verbose YOLO logging..."
        if ! grep $grep_options "$disabled_verbose_code" "$predictor_py_path"; then
            $verbose && echo "Disable verbose line not found. Attempting to add line..."
            # Add the line
            sed $sed_options "/LOGGER.info(\"\")/a\\$disabled_verbose_code" "$predictor_py_path"
            # Check if line was successfully added
            if grep $grep_options "$disabled_verbose_code" "$predictor_py_path"; then
                # Line was successfully added
                $verbose && echo "Line added!"
            else
                # Line was not found
                # Non-verbose error
                echo "Failed to add disable verbose logging line to predictor.py!"
            fi
        else
            $verbose && echo "Disable verbose line already exists. Nothing to change. Continuing..."
        fi
    fi
else
    echo "Could not find predictor.py. Please pip install before running the install script."
    exit 1
fi

cd "$grace_dir" || exit
exit 0