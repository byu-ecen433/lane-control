#!/usr/bin/env python3
"""
ECEN 433 - Lab 5
================

Live view of the simulated robot. On top, a bird's-eye view of the lane that
scrolls with the robot. Below, d, phi and your omega over time.

Thin gray traces are the noisy pose your node receives. Thick traces are where
the robot really is. Red marks are the simulator's kicks.
"""

import bisect
import math
import threading

import matplotlib
import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped
from geometry_msgs.msg import Pose2D

# Duckietown road geometry, from the lane filter's defaults (m).
LANE_WIDTH = 0.23
YELLOW_WIDTH = 0.025
WHITE_WIDTH = 0.05
DASH_LENGTH = 0.05

# Duckiebot footprint relative to the wheel axle (m): (forward, left).
ROBOT_OUTLINE = [(-0.05, -0.065), (0.10, -0.065), (0.14, 0.0), (0.10, 0.065), (-0.05, 0.065)]

VIEW_BEHIND = 2.0
VIEW_AHEAD = 0.4


class LaneGraph:
    def __init__(self):
        self.window = rospy.get_param("~window", 30.0)
        self.lock = threading.Lock()
        self.start = None
        self.meas_t, self.meas_d, self.meas_phi = [], [], []
        self.true_t, self.true_s, self.true_d, self.true_phi = [], [], [], []
        self.cmd_t, self.omega = [], []
        self.kicks = []  # (t, s, d)
        self.kicked = False
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.meas_cb, queue_size=10)
        rospy.Subscriber("lane_sim_node/true_pose", Pose2D, self.true_cb, queue_size=10)
        rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.cmd_cb, queue_size=10)

    def now(self):
        t = rospy.get_time()
        if self.start is None:
            self.start = t
        return t - self.start

    def meas_cb(self, msg):
        with self.lock:
            self.meas_t.append(self.now())
            self.meas_d.append(msg.d)
            self.meas_phi.append(msg.phi)

    def true_cb(self, msg):
        # The simulator sends an all-NaN pose just before each kick. Keeping it
        # breaks the lines, so the jump isn't drawn as driving.
        with self.lock:
            t = self.now()
            if self.kicked:
                self.kicks.append((t, msg.x, msg.y))
            self.kicked = math.isnan(msg.x)
            self.true_t.append(t)
            self.true_s.append(msg.x)
            self.true_d.append(msg.y)
            self.true_phi.append(msg.theta)

    def cmd_cb(self, msg):
        with self.lock:
            self.cmd_t.append(self.now())
            self.omega.append(msg.omega)

    def recent(self, t, *series):
        """The last `window` seconds of t and each series."""
        i = bisect.bisect_left(t, t[-1] - self.window) if t else 0
        return [s[i:] for s in (t,) + series]


def robot_polygon(s, d, phi):
    c, sn = math.cos(phi), math.sin(phi)
    return [(s + fx * c - fy * sn, d + fx * sn + fy * c) for fx, fy in ROBOT_OUTLINE]


def yellow_dashes(s_min, s_max):
    """Center line dashes covering [s_min, s_max], fixed to the road so they scroll."""
    period = 2 * DASH_LENGTH
    first = math.floor(s_min / period) * period
    n = int((s_max - first) / period) + 1
    return [(first + period * i, DASH_LENGTH) for i in range(n)]


def main():
    rospy.init_node("lane_graph")
    output_file = rospy.get_param("~output_file", "")
    if rospy.get_param("~only_output_to_file", False):
        matplotlib.use("pdf")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    graph = LaneGraph()

    fig = plt.figure(figsize=(9, 9))
    grid = fig.add_gridspec(4, 1, height_ratios=[1.3, 1, 1, 1])
    ax_lane = fig.add_subplot(grid[0])
    ax_d = fig.add_subplot(grid[1])
    ax_phi = fig.add_subplot(grid[2], sharex=ax_d)
    ax_omega = fig.add_subplot(grid[3], sharex=ax_d)

    # d is positive to the left, so with the robot driving left to right the
    # yellow center line is on top and the white edge line on the bottom.
    half = LANE_WIDTH / 2
    ax_lane.set_facecolor("0.3")
    ax_lane.axhspan(-half - WHITE_WIDTH, -half, color="white", zorder=1)
    ax_lane.axhline(0.0, color="0.65", linestyle=":", linewidth=1, zorder=1)
    ax_lane.set_xlim(-VIEW_BEHIND, VIEW_AHEAD)
    ax_lane.set_ylim(-half - WHITE_WIDTH - 0.03, half + YELLOW_WIDTH + 0.05)
    ax_lane.set_aspect("equal")
    ax_lane.set_xlabel("distance along the lane (m)")
    ax_lane.set_ylabel("d (m)")
    ax_lane.set_title("Your robot in the lane")
    dashes = None
    trail, = ax_lane.plot([], [], color="tab:blue", linewidth=2, zorder=3)
    kick_marks, = ax_lane.plot([], [], "x", color="tab:red", markersize=8, mew=2, zorder=4)
    robot = Polygon(robot_polygon(0, 0, 0), closed=True, facecolor="tab:orange",
                    edgecolor="k", zorder=5)
    ax_lane.add_patch(robot)

    meas_d, = ax_d.plot([], [], color="0.6", linewidth=0.8, label="what your node sees")
    true_d, = ax_d.plot([], [], color="tab:blue", linewidth=2, label="true")
    meas_phi, = ax_phi.plot([], [], color="0.6", linewidth=0.8)
    true_phi, = ax_phi.plot([], [], color="tab:green", linewidth=2)
    line_omega, = ax_omega.plot([], [], color="tab:red", linewidth=1.2)
    ax_d.legend(loc="upper right", fontsize="small")
    for ax in (ax_d, ax_phi, ax_omega):
        ax.axhline(0.0, color="k", linewidth=0.5)
        ax.grid(True, alpha=0.3)
    ax_d.set_ylabel("d (m)")
    ax_phi.set_ylabel("phi (rad)")
    ax_omega.set_ylabel("omega (rad/s)")
    ax_omega.set_xlabel("time (s)")
    fig.tight_layout()
    kicks_drawn = 0

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        with graph.lock:
            mt, md, mp = graph.recent(graph.meas_t, graph.meas_d, graph.meas_phi)
            tt, ts, td, tp = graph.recent(graph.true_t, graph.true_s, graph.true_d, graph.true_phi)
            ct, om = graph.recent(graph.cmd_t, graph.omega)
            kicks = list(graph.kicks)

        meas_d.set_data(mt, md)
        meas_phi.set_data(mt, mp)
        true_d.set_data(tt, td)
        true_phi.set_data(tt, tp)
        line_omega.set_data(ct, om)

        if ts and not math.isnan(ts[-1]):
            s, d, phi = ts[-1], td[-1], tp[-1]
            ax_lane.set_xlim(s - VIEW_BEHIND, s + VIEW_AHEAD)
            if dashes is not None:
                dashes.remove()
            dashes = ax_lane.broken_barh(yellow_dashes(s - VIEW_BEHIND, s + VIEW_AHEAD),
                                         (half, YELLOW_WIDTH), color="gold", zorder=1)
            trail.set_data(ts, td)
            robot.set_xy(robot_polygon(s, d, phi))
        kick_marks.set_data([k[1] for k in kicks], [k[2] for k in kicks])

        for kt, _, _ in kicks[kicks_drawn:]:
            for ax in (ax_d, ax_phi, ax_omega):
                ax.axvline(kt, color="tab:red", linestyle="--", linewidth=0.8)
        kicks_drawn = len(kicks)

        if mt:
            ax_d.set_xlim(max(0.0, mt[-1] - graph.window), max(mt[-1], 1.0))
        for ax in (ax_d, ax_phi, ax_omega):
            ax.relim()
            ax.autoscale_view(scalex=False)

        if output_file:
            fig.savefig(output_file)
        plt.pause(0.05)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
