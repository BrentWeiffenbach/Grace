# Grace

Key Launch Files:

* amcl.launch launches an amcl demonstration using a map_file arg, default is a presaved room map from sim
* explore.launch UNDERCONSTRUCTION WILL BE FOR FRONTIER NAVIGATION
* gmapping.launch a recreation of the gmapping launch file from the move_base navigation stack with sim args
* grace.launch the main launch file to launch everything as needed, as arguments for sim, gmapping, teleop, explore, amcl, and map_file. Ex. roslaunch grace.launch sim:=true gmapping:=true    Would launch a simulated turtlebot in a room example and begin gmapping, you can use 2D nav goals to have it move around the room and if you save the map afterwards you can run roslaunch grace.launch sim:=true amcl:=true    to navigate through the saved map using amcl

Everything so far has been built from learn.turtlebot.com and [turtlebot_navigation](http://wiki.ros.org/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot)

To get the sim world used in testing clone: <https://github.com/aws-robotics/aws-robomaker-small-house-world>

TODO:
Make a requirements.txt
Proper frontier navigation
YOLO implementation
YOLO detections in rviz
implementation onto navigation stack (move_base and amcl)

## Installation

1. Install python 3.8
    * `sudo apt install python3.8.0`
2. Go to your catkin_ws/src, and run `git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git`
3. Go to catkin_ws, and run `catkin_make`
4. `sudo apt-get install ros-melodic-dwa-local-planner`
5. `sudo apt-get install ros-melodic-ros-numpy`
6. Change numpy to use float64
    * Go to [yolo_detect.py](scripts/yolo_detect.py), and control click on `ros_numpy`. Then, control click on `point_cloud2`.
    * In `point_cloud2.py`, find `get_xyz_points()`, change `dtype` to `dtype=np.float64`. Saving will require a sudo password.
7. Run the install script located at [install/install.sh](install/install.sh).

## Usage

To launch the turtlebot navigation in sim, run `roslaunch grace grace.launch gmapping:=true`. Find the other arguments in the [grace](launch/grace.launch) launch file

## Known Issues

A known bug is that `/opt/ros/melodic/include/gmapping/gridfastslam/gridslamprocessor.hxx` floods the console without the ability to disable.
