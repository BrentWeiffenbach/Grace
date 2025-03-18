# Grace

Key Launch Files:

* gmapping.launch a recreation of the gmapping launch file from the move_base navigation stack with sim args
* grace.launch the main launch file to launch everything as needed, as arguments for sim, gmapping, and map_file. Ex. `roslaunch grace.launch sim:=true` Would launch a simulated turtlebot in a room example and begin gmapping, you can use 2D nav goals to have it move around the room. You can run it headless by providing `headless:=true`, which will hide Gazebo.

Everything so far has been built from learn.turtlebot.com and [turtlebot_navigation](http://wiki.ros.org/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot)

TODO:
Make a requirements.txt for python 2.7

## Installation

1. Install python 3.8
    * `sudo apt install python3.8.0`
2. Run the install script located at [install/install.sh](install/install.sh).

## Usage

To launch the turtlebot navigation in sim, run `roslaunch grace.launch`. Find the other arguments in the [grace](launch/grace.launch) launch file.

You can also set the output to verbose by using the `verbose:=true` argument.

## Known Issues

* A known bug is that `/opt/ros/melodic/include/gmapping/gridfastslam/gridslamprocessor.hxx` floods the console without the ability to disable.
