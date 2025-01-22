# The turtlebot is waiting for a command
int32 WAITING = 0

# The turtlebot is exploring
int32 EXPLORING = 1

# The turtlebot is picking an object
int32 PICKING = 2

# The turtlebot is placing an object
int32 PLACING = 3

# The turtlebot is going to its zero location
int32 HOMING = 4

# An int representing the class states of the turtlebot. Valid options are:
# `RobotState.WAITING`
# `RobotState.EXPLORING`
# `RobotState.PICKING`
# `RobotState.PLACING`
# `RobotState.HOMING`
int32 state