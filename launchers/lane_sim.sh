#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# Your controller against the lane simulator, with a live plot.
# Run it from outside the container with:
#
#     dts devel run -X -L lane_sim
#
# -X is what lets the plot window out of the container.
dt-exec roslaunch pid_control lane_control.launch test:=true

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
