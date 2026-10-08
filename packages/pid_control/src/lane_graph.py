#!/usr/bin/env python3
"""
ECEN 433 - Lab 5
================

Live plot of the lane pose your controller sees and the omega it commands.
The pose is the noisy one your node receives; the simulator's log reports the
true pose.
"""

import bisect
import threading

import matplotlib
import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped


class LaneGraph:
    def __init__(self):
        self.window = rospy.get_param("~window", 30.0)
        self.lock = threading.Lock()
        self.start = None
        self.pose_t, self.d, self.phi = [], [], []
        self.cmd_t, self.omega = [], []
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.pose_cb, queue_size=10)
        rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.cmd_cb, queue_size=10)

    def now(self):
        t = rospy.get_time()
        if self.start is None:
            self.start = t
        return t - self.start

    def pose_cb(self, msg):
        with self.lock:
            self.pose_t.append(self.now())
            self.d.append(msg.d)
            self.phi.append(msg.phi)

    def cmd_cb(self, msg):
        with self.lock:
            self.cmd_t.append(self.now())
            self.omega.append(msg.omega)

    def snapshot(self, t, *series):
        """The last `window` seconds of t and each series."""
        i = bisect.bisect_left(t, t[-1] - self.window) if t else 0
        return [s[i:] for s in (t,) + series]


def main():
    rospy.init_node("lane_graph")
    output_file = rospy.get_param("~output_file", "")
    if rospy.get_param("~only_output_to_file", False):
        matplotlib.use("pdf")
    import matplotlib.pyplot as plt

    graph = LaneGraph()

    fig, (ax_d, ax_phi, ax_omega) = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    line_d, = ax_d.plot([], [], "b-")
    line_phi, = ax_phi.plot([], [], "g-")
    line_omega, = ax_omega.plot([], [], "r-")
    for ax in (ax_d, ax_phi, ax_omega):
        ax.axhline(0.0, color="k", linewidth=0.5)
        ax.grid(True, alpha=0.3)
    ax_d.set_ylabel("d (m)")
    ax_phi.set_ylabel("phi (rad)")
    ax_omega.set_ylabel("omega (rad/s)")
    ax_omega.set_xlabel("time (s)")
    ax_d.set_title("Lane pose and control")
    fig.tight_layout()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        with graph.lock:
            pose_t, d, phi = graph.snapshot(graph.pose_t, graph.d, graph.phi)
            cmd_t, omega = graph.snapshot(graph.cmd_t, graph.omega)
        line_d.set_data(pose_t, d)
        line_phi.set_data(pose_t, phi)
        line_omega.set_data(cmd_t, omega)
        for ax in (ax_d, ax_phi, ax_omega):
            ax.relim()
            ax.autoscale_view()
        if output_file:
            fig.savefig(output_file)
        plt.pause(0.05)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
