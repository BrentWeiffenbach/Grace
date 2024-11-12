# These are some commands I had to run to install the first time. Uncomment them if there are issues with the script to see if it fixes things.
# sudo apt-get update
# sudo apt install rosbash
# sudo apt-get install ros-melodic-gazebo-ros gazebo9 ros-melodic-robot-state-publisher ros-melodic-xacro
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

for python_file in "scripts/${python_files[@]}"; do
    if [[ -f "$python_file" ]]; then
        
        # Check first line of the file
        first_line=$(head -n 1 "$python_file")

        # Ensure the first line starts with a shebang
        if [[ "$first_line" =~ ^#! ]]; then
            echo "Changing shebang in $python_file..."

            # Change the first line to be the path
            # 1s is find the first line
            # ^#!.*$ is a regex for the entire line if it starts with #!
            # #!$python_path is the shebang to replace it with
            sed -i "1s|^#!.*$|#!$python_path|" "$python_file"
            echo "Changed shebang in $python_file."
        else
            echo "$python_file does not start with a shebang. Skipping..."
        fi
    else
        echo "$python_file does not exist. Skipping..."
    fi
done

echo "Done changing shebangs."