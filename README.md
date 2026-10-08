# ECEN 433 - Lab 5: PID Control & Lane Following

Starter code for Lab 5. You will write one ROS node, a PID lane controller,
tune it against a simulated Duckiebot on this machine, then run the same node
on your robot, where it replaces Duckietown's controller and closes the loop
around your Lab 4 detector.

The full lab instructions live on the course site. This README covers the
repository itself: how it is laid out, and how to build and run it.


## Building and running

Build the Docker image. Run this from the root of this repository:

```bash
dts devel build -f
```

Then, on this machine, against the simulator:

```bash
dts devel run -X -L lane_sim
```

- `-X` allows the container to open GUI windows. Without it the plot never
  appears.
- `-L lane_sim` runs `launchers/lane_sim.sh`.

On your robot, alongside your Lab 4 stack:

```bash
dts devel build -H DUCKIEBOT_NAME -f
dts devel run   -H DUCKIEBOT_NAME -L lane_control
```

No `-X` there - the robot has no screen. Plot from a separate
`dts gui DUCKIEBOT_NAME` shell with `rqt_plot`.

To poke around inside the container instead of launching straight away:

```bash
dts devel run -X --cmd bash
```

and to attach a second terminal to a container that is already running:

```bash
dts devel run attach
```

New Python nodes must be executable or ROS will not find them:

```bash
chmod +x packages/pid_control/src/<node_name>.py
```
