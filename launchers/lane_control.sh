#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# Your controller on the robot, fed by your Lab 4 stack. THIS ONE RUNS ON THE BOT:
#
#     dts devel build -H DUCKIEBOT_NAME -f
#     dts devel run   -H DUCKIEBOT_NAME -L lane_control
#
# Start your Lab 4 lane_following launcher first, with Duckietown's controller off.
dt-exec roslaunch pid_control lane_control.launch test:=false

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
