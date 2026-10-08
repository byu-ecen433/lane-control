#!/usr/bin/env python3
"""
ECEN 433 - Lab 5
================

A simulated Duckiebot in a lane, for tuning your controller without the robot.

It publishes LanePose on lane_filter_node/lane_pose and drives with whatever
Twist2DStamped arrives on lane_controller_node/car_cmd. Those are the same
relative topics as on the robot, so your node cannot tell the two apart.

Every `kick_period` seconds the robot is knocked to the next (d, phi) in
`kicks`, and how your controller recovered from the last one is logged:
how long until |d| < d_tol and |phi| < phi_tol for good, how far d crossed
past the center line, and the largest omega your node asked for. These are
measured on the true pose, not the noisy one you are sent.
"""

import math
import random
from collections import deque

import rospy
from duckietown_msgs.msg import LanePose, Twist2DStamped

DEFAULT_KICKS = [[0.10, 0.0], [-0.08, 0.4], [0.0, -0.5], [0.06, -0.3]]


class KickReport:
    """Settling time, overshoot and peak omega for one kick."""

    def __init__(self, index, d0, phi0, d_tol, phi_tol):
        self.index = index
        self.d0, self.phi0 = d0, phi0
        self.d_tol, self.phi_tol = d_tol, phi_tol
        # The side the robot starts on, or drifts to first for a pure heading
        # kick. Crossing to the other side of center is overshoot.
        self.side = math.copysign(1.0, d0 if d0 != 0 else phi0)
        self.settled_at = None
        self.overshoot = 0.0
        self.max_omega = 0.0

    def update(self, t, d, phi, omega):
        self.overshoot = max(self.overshoot, -self.side * d)
        self.max_omega = max(self.max_omega, abs(omega))
        inside = abs(d) < self.d_tol and abs(phi) < self.phi_tol
        if not inside:
            self.settled_at = None
        elif self.settled_at is None:
            self.settled_at = t

    def log(self):
        settle = ("did not settle" if self.settled_at is None
                  else f"settled in {self.settled_at:.1f} s")
        rospy.loginfo(
            "kick %d (d=%+.2f m, phi=%+.2f rad): %s, overshoot %.3f m, max |omega| %.1f rad/s",
            self.index, self.d0, self.phi0, settle, self.overshoot, self.max_omega)


class LaneSimNode:
    def __init__(self):
        rospy.init_node("lane_sim_node")
        self.rate_hz = rospy.get_param("~rate", 15.0)
        self.delay_steps = int(rospy.get_param("~delay_steps", 2))
        self.noise_d = rospy.get_param("~noise_d", 0.01)
        self.noise_phi = rospy.get_param("~noise_phi", 0.05)
        self.omega_max = rospy.get_param("~omega_max", 8.0)
        self.v_max = rospy.get_param("~v_max", 1.0)
        self.kick_period = rospy.get_param("~kick_period", 10.0)
        self.kicks = rospy.get_param("~kicks", DEFAULT_KICKS)
        self.d_tol = rospy.get_param("~d_tol", 0.02)
        self.phi_tol = rospy.get_param("~phi_tol", 0.2)

        self.d, self.phi = self.kicks[0]
        self.kick_index = 0
        self.kick_time = None
        self.report = None

        # Commands take effect delay_steps ticks after they arrive, roughly the
        # camera-to-wheels latency on the real robot.
        self.latest_cmd = (0.0, 0.0)
        self.commanded_omega = 0.0
        self.cmd_queue = deque([(0.0, 0.0)] * (self.delay_steps + 1),
                               maxlen=self.delay_steps + 1)

        self.pub_pose = rospy.Publisher("lane_filter_node/lane_pose", LanePose, queue_size=1)
        rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.cmd_cb, queue_size=1)

        rospy.loginfo("lane_sim_node publishing on %s, waiting for commands on %s",
                      rospy.resolve_name("lane_filter_node/lane_pose"),
                      rospy.resolve_name("lane_controller_node/car_cmd"))

    def cmd_cb(self, msg):
        self.commanded_omega = msg.omega
        v = max(-self.v_max, min(self.v_max, msg.v))
        omega = max(-self.omega_max, min(self.omega_max, msg.omega))
        self.latest_cmd = (v, omega)
        if self.kick_time is None:
            rospy.loginfo("first command received, starting the kicks")
            self.start_kick(rospy.get_time())

    def start_kick(self, now):
        if self.report is not None:
            self.report.log()
        self.d, self.phi = self.kicks[self.kick_index % len(self.kicks)]
        self.report = KickReport(self.kick_index % len(self.kicks) + 1,
                                 self.d, self.phi, self.d_tol, self.phi_tol)
        self.kick_index += 1
        self.kick_time = now

    def step(self, dt):
        self.cmd_queue.append(self.latest_cmd)
        v, omega = self.cmd_queue[0]

        self.d += v * math.sin(self.phi) * dt
        self.phi += omega * dt
        self.phi = math.atan2(math.sin(self.phi), math.cos(self.phi))

        now = rospy.get_time()
        if self.kick_time is not None:
            self.report.update(now - self.kick_time, self.d, self.phi, self.commanded_omega)
            if now - self.kick_time >= self.kick_period:
                self.start_kick(now)

    def publish_pose(self):
        msg = LanePose()
        msg.header.stamp = rospy.Time.now()
        msg.d = self.d + random.gauss(0.0, self.noise_d)
        msg.phi = self.phi + random.gauss(0.0, self.noise_phi)
        msg.in_lane = abs(self.d) < 0.2
        msg.status = LanePose.NORMAL
        self.pub_pose.publish(msg)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step(1.0 / self.rate_hz)
            self.publish_pose()
            rate.sleep()


if __name__ == "__main__":
    try:
        LaneSimNode().run()
    except rospy.ROSInterruptException:
        pass
