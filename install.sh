# These are some commands I had to run to install the first time. Uncomment them if there are issues with the script to see if it fixes things.
# sudo apt-get update
# sudo apt install rosbash
# sudo apt-get install ros-melodic-gazebo* ros-melodic-robot-state-publisher ros-melodic-xacro
# sudo apt install python3.8.0

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

echo "Changing shebangs..."

# Declare files that use relative shebangs here:
python_files=("yolo_detect.py")

for python_file in "${python_files[@]}"; do
    full_path="scripts/$python_file"
    if [[ -f "$full_path" ]]; then
        
        # Check first line of the file
        first_line=$(head -n 1 "$full_path")

        # Ensure the first line starts with a shebang
        if [[ "$first_line" =~ ^#! ]]; then
            echo "Changing shebang in $full_path..."

            # Change the first line to be the path
            # 1s is find the first line
            # ^#!.*$ is a regex for the entire line if it starts with #!
            # #!$python_path is the shebang to replace it with
            sed -i "1s|^#!.*$|#!$python_path|" "$full_path"
            echo "Changed shebang in $full_path."
        else
            echo "$full_path does not start with a shebang. Skipping..."
        fi
    else
        echo "$full_path does not exist. Skipping..."
    fi
done

echo "Done changing shebangs."

##############################
# Change other hardcoded paths
##############################

# Images in yaml
yaml_paths=("my_map" "tutorial") # Must end in yaml

echo "Changing yaml file hardcoded paths..."
for yaml_file in "${yaml_paths[@]}"; do
    full_path="maps/$yaml_file.yaml"
    
    if [[ -f $full_path ]]; then
        sed -i "s|image: .*|image: $install_dir/maps/$yaml_file.pgm|" $full_path
    else
        echo "$full_path not found. Skipping..."
    fi
done
