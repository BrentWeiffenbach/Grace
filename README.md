# Grace

Mobile manipulation enables robots to perform complex tasks using a mobile base and one or more manipulators. The TurtleBot2 is an open-source mobile base platform that uses a RGB-D camera for exploration and navigation. Our project, GRACE, is a mobile manipulator that adds a 3D-printed 6-DOF arm to the TurtleBot2 to perform common household errands. Our work equips GRACE with software for simultaneous localization and mapping (SLAM), perception of semantic objects using YOLO object detection, and a vision-guided pick-and-place arm with custom motion planning and control. GRACE navigates unknown environments, searches for a target object, picks up the object, and navigates to a semantic goal location to place the object.

![The poster used for the WPI URPS](<figures/poster.png>)

GRACE is a Major Qualifying Project completed at Worcester Polytechnic Institute in the 2024-2025 school year.

See the [YouTube video](https://www.youtube.com/watch?v=ce3OphPrwlA) to learn more about the project, and the link to our paper coming soon!

## Installation

1. Install python 3.8
    * `sudo apt install python3.8.0`
2. Run the install script located at [install/install.sh](install/install.sh).

## Setting up the Turtlebot

<!-- This is cursed. HTML in markdown. -->
<div align="center">
  <img src="figures/grace.png" alt="A photo of GRACE" width="175"/>
</div>

1. Follow the instructions at [learn.turtlebot.com](https://learn.turtlebot.com) and [turtlebot_navigation](http://wiki.ros.org/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot)

2. Ensure that the Kinect is plugged into a USB 2.0 port (NOT USB 3.0), as well as the Kobuki base (with the mulex connectors right above the B0 and B1 buttons).

## Usage

* To launch the turtlebot navigation in sim, run `roslaunch grace.launch`. Find the other arguments in the [grace](launch/grace.launch) launch file.

* Run the onboard on the TurtleBot by running `roslaunch onboard.launch`.

* `grace.launch` is the main launch file to launch everything as needed. Use this file to run simulations, or to launch GRACE's workstation computer. This file is responsible for launching the sim enviornment if in sim, the joint state publisher, robot state publisher, arm MoveIt move_group, arm and base execution controllers, grasping node, navigation, ad the master node. Using this file involves many args depending on what application is desired. Refer to grace.launch to read more about the potential arguments. Typically this file is run after the `onboard.launch` file is run from the onboard computer on the TurtleBot2.

* onboard.launch is run from the TurtleBot2, you can ssh into your onboard machine and launch the file from there. This launch file manages the kinect sensor, gmapping, and low-level arduino controller.

* If you want to run in sim, do not run the onboard launch file. Instead, run `grace.launch` with `sim:=true`. If you have the arm nearby, plug in the Arduino and you can enable `arm:=true`. Otherwise, run it with `arm:=false`. Because gmapping floods the console and makes debugging hard, we typically launch the custom gmapping file in grace_navigation in a seperate terminal for readabilitty

* To visualize the YOLO detections in RVIZ, run grace.launch with `verbose:=true`.

## Known Issues

* A known bug is that `/opt/ros/melodic/include/gmapping/gridfastslam/gridslamprocessor.hxx` floods the console without the ability to disable.
  * Fixed by moving movebase to a different launch file

* An error when running the arm involving `/dev/tty0`
  * You can fix this by unplugging ALL the USB devices from the onboard, then plugging in the Arduino first, then plugging in the rest of the devices.