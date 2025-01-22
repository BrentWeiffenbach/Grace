# These are some commands I had to run to install the first time. Uncomment them if there are issues with the script to see if it fixes things.
# sudo apt-get update
# sudo apt install rosbash
# sudo apt-get install ros-melodic-gazebo* ros-melodic-robot-state-publisher ros-melodic-xacro
# sudo apt install python3.8.0

# A flag to toggle verbose logging to the terminal
verbose=false # true or fase

# Check if venv exists
if  [[ ! -d yolovenv ]]; then
    echo "yolovenv not found. Installing..."

    # Note: the line below is UNTESTED, so if it doesn't work, just run the command that is in the README...
    python3.8 -m virtualenv -p python3.8 yolovenv
    echo "yolovenv created."
fi

#####################
# Change the shebangs
#####################
# Get the full python path for the shebang here
install_dir="$(cd "$(dirname "$0")" && pwd)"
python_path="$install_dir/yolovenv/bin/python"

if [[ ! -f "$python_path" ]]; then
    echo "Could not find python path."
    exit 1
fi

$verbose && echo "Changing shebangs..."

# Declare files that use relative shebangs here:
python_files=("yolo_detect.py" "add_marker.py" "ultralytics_patch.py" "grace_node.py")

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
        sed -i "s|image: .*|image: $install_dir/maps/$yaml_file.pgm|" $full_path
    else
        $verbose && echo "$full_path not found. Skipping..."
    fi
done

#################################
# Disable verbose logging in YOLO
#################################
# Change the below line to toggle between verbose YOLO logging
do_verbose_yolo_logging=false

predictor_py_path="$install_dir/yolovenv/lib/python3.8/site-packages/ultralytics/engine/predictor.py"
if [[ -f "$predictor_py_path" && -r "$predictor_py_path" && -w "$predictor_py_path" ]]; then
    disabled_verbose_code="            self.args.verbose = False"
    grep_options=""
    if [ "$verbose" == "false" ]; then
        grep_options="-q "
    fi
    sed_options="-i "

    if [ "$do_verbose_yolo_logging" == "true" ]; then # Disable YOLO logging (remove supressing line)
        $verbose && echo "Attempting to enable verbose YOLO logging..."
        # Check if it was disabled before
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
    echo "Skipping this section..."
fi

# TODO: Add verbosity to this
[[ -d out/SLAM ]] || mkdir out/SLAM

exit 0 # Successfully exit