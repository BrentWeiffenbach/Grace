Key Launch Files:
* amcl.launch launches an amcl demonstration using a map_file arg, default is a presaved room map from sim
* explore.launch UNDERCONSTRUCTION WILL BE FOR FRONTIER NAVIGATION
* gmapping.launch a recreation of the gmapping launch file from the move_base navigation stack with sim args
* grace.launch the main launch file to launch everything as needed, as arguments for sim, gmapping, teleop, explore, amcl, and map_file. Ex. roslaunch grace.launch sim:=true gmapping:=true    Would launch a simulated turtlebot in a room example and begin gmapping, you can use 2D nav goals to have it move around the room and if you save the map afterwards you can run roslaunch grace.launch sim:=true amcl:=true    to navigate through the saved map using amcl

Everything so far has been built from learn.turtlebot.com and [turtlebot_navigation](http://wiki.ros.org/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot)

TODO:
Make a requirements.txt
Proper frontier navigation
YOLO implementation
YOLO detections in rviz
implementation onto navigation stack (move_base and amcl)