#!/usr/bin/env python3
"""
Potential field mobile robot controller — ROS2 / Gazebo Harmonic
with real-time graphs:
  - Window 1: Min obstacle distance + distance to goal vs time
  - Window 2: Attractive force magnitude + repulsive force magnitude vs time

DEPLOY:
  cp controller_final.py ~/ws_mobile/src/mobile_robot/mobile_robot/controller_final.py
  cd ~/ws_mobile
  colcon build --packages-select mobile_robot --symlink-install
  source install/setup.bash
  ros2 run mobile_robot controller_final
"""

import math
import time
import threading
import os
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

topic1 = '/cmd_vel'
topic2 = '/odom1'
topic3 = '/scan'

HISTORY = 200   # ~10 seconds at 20 Hz


class ControllerNode(Node):

    def __init__(self, xdu, ydu, kau, kru, kthetatu, gstaru,
                 eps_orientu, eps_controlu):
        super().__init__('controller_node')

        self.xdp         = xdu
        self.ydp         = ydu
        self.kap         = kau
        self.krp         = kru
        self.kthetap     = kthetatu
        self.gstarp      = gstaru
        self.eps_orient  = eps_orientu
        self.eps_control = eps_controlu

        self.window_size           = 10
        self.stuck_dist_threshold  = 0.03
        self.stuck_count_threshold = 15
        self.escape_speed          = 0.7
        self.escape_min_travel     = 0.80
        self.post_escape_cooldown  = 60
        self.f_norm_min_resume     = 0.3
        self.near_goal_suppress    = 3.0

        self._pos_history      = []
        self._stuck_counter    = 0
        self._escaping         = False
        self._escape_angle     = 0.0
        self._escape_start_x   = 0.0
        self._escape_start_y   = 0.0
        self._cooldown_counter = 0
        self._last_escape_x    = None
        self._last_escape_y    = None
        self._escape_flip      = False

        self._last_sim_time      = None
        self._last_sim_wall_time = time.time()
        self._sim_frozen         = False

        # ── Graph data (all thread-safe deques) ───────────────────────────
        self.graph_lock      = threading.Lock()
        self.g_time          = deque(maxlen=HISTORY)
        self.g_min_dist      = deque(maxlen=HISTORY)   # nearest obstacle (m)
        self.g_dist_goal     = deque(maxlen=HISTORY)   # dist to goal (m)
        self.g_af_norm       = deque(maxlen=HISTORY)   # |attractive force|
        self.g_rf_norm       = deque(maxlen=HISTORY)   # |repulsive force|
        self.g_net_norm      = deque(maxlen=HISTORY)   # |net force|
        self.graph_start     = time.time()

        self.OdometryMsg     = Odometry()
        self.LidarMsg        = LaserScan()
        self.initialTime     = time.time()
        self.msgOdometryTime = time.time()
        self.msgLidarTime    = time.time()
        self.controlVel      = Twist()

        self.has_goal = False
        self.ControlPublisher = self.create_publisher(Twist, topic1, 10)
        self.GoalSubscriber   = self.create_subscription(
            PoseStamped, '/goal_pose', self.GoalCallback, 10)
        self.PoseSubscriber   = self.create_subscription(
            Odometry,  topic2, self.SensorCallbackPose,  10)
        self.LidarSubscriber  = self.create_subscription(
            LaserScan, topic3, self.SensorCallbackLidar, 10)

        self.period = 0.05
        self.timer  = self.create_timer(self.period, self.ControlFunction)

    def SensorCallbackPose(self, msg):
        self.OdometryMsg     = msg
        self.msgOdometryTime = time.time()
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_sim_time is None or sim_t != self._last_sim_time:
            self._last_sim_time      = sim_t
            self._last_sim_wall_time = time.time()

    def SensorCallbackLidar(self, msg):
        self.LidarMsg    = msg
        self.msgLidarTime = time.time()

    def GoalCallback(self, msg):
        self.xdp = msg.pose.position.x
        self.ydp = msg.pose.position.y
        self.has_goal = True
        print(f"[RViz2] New Goal Received: ({self.xdp:.2f}, {self.ydp:.2f})")

    def orientationError(self, theta_, thetad_):
        if (thetad_ > np.pi/2) and (thetad_ <= np.pi):
            if (theta_ > -np.pi) and (theta_ <= -np.pi/2):
                theta_ += 2*np.pi
        if (theta_ > np.pi/2) and (theta_ <= np.pi):
            if (thetad_ > -np.pi) and (thetad_ <= -np.pi/2):
                thetad_ += 2*np.pi
        return thetad_ - theta_

    def _normal_control(self, F, F_norm, theta):
        lidar = np.array(self.LidarMsg.ranges)
        valid = lidar[~np.isinf(lidar) & ~np.isnan(lidar)]
        if len(valid) > 0 and np.min(valid) < 0.35:
            F_norm = min(F_norm, 0.15)
        thetaD  = math.atan2(float(F[1, 0]), float(F[0, 0]))
        eorient = self.orientationError(theta, thetaD)
        if abs(eorient) > abs(self.eps_orient):
            thetavel = self.kthetap * eorient
            xvel     = 0.08
        else:
            thetavel = self.kthetap * eorient
            xvel     = min(F_norm, 2.5)
            if 0.0 < xvel < 0.08:
                xvel = 0.08
        return thetavel, xvel

    def _goal_direct_control(self, x, y, theta, xd, yd):
        goal_ang = math.atan2(yd - y, xd - x)
        eorient  = self.orientationError(theta, goal_ang)
        dist     = math.hypot(x - xd, y - yd)
        if abs(eorient) > abs(self.eps_orient):
            thetavel = self.kthetap * eorient
            xvel     = 0.08
        else:
            thetavel = self.kthetap * eorient
            xvel     = min(dist * 0.5, 0.8)
            if 0.0 < xvel < 0.08:
                xvel = 0.08
        return thetavel, xvel

    def ControlFunction(self):
        if not self.has_goal:
            self.controlVel.linear.x  = 0.0
            self.controlVel.angular.z = 0.0
            self.ControlPublisher.publish(self.controlVel)
            return

        ka     = self.kap
        kr     = self.krp
        gstar  = self.gstarp
        xd, yd = self.xdp, self.ydp

        x = self.OdometryMsg.pose.pose.position.x
        y = self.OdometryMsg.pose.pose.position.y
        quat = self.OdometryMsg.pose.pose.orientation
        (_, _, theta) = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])

        LidarRanges     = np.array(self.LidarMsg.ranges)
        angle_min       = self.LidarMsg.angle_min
        angle_increment = self.LidarMsg.angle_increment

        # frozen sim check
        if time.time() - self._last_sim_wall_time > 2.0:
            if not self._sim_frozen:
                print("[SIM FROZEN] stopping motors.")
                self._sim_frozen = True
            self.controlVel.linear.x  = 0.0
            self.controlVel.angular.z = 0.0
            self.ControlPublisher.publish(self.controlVel)
            return
        else:
            if self._sim_frozen:
                print("[SIM RESUMED]")
                self._sim_frozen       = False
                self._stuck_counter    = 0
                self._pos_history.clear()
                self._cooldown_counter = 20

        # attractive force
        vectorD      = np.array([[x - xd], [y - yd]])
        AF           = -ka * vectorD
        dist_to_goal = float(np.linalg.norm(vectorD))
        af_norm      = float(np.linalg.norm(AF))

        # repulsive force
        indices_not_inf = np.where(
            ~np.isinf(LidarRanges) & ~np.isnan(LidarRanges))[0]
        obstacleYES = len(indices_not_inf) > 0

        RF         = np.zeros((2, 1))
        min_dists  = []
        min_angles = []

        if obstacleYES:
            diff_arr  = np.diff(indices_not_inf)
            split_idx = np.where(np.abs(diff_arr) > 1)[0] + 1
            parts     = np.split(indices_not_inf, split_idx)
            for part in parts:
                ranges_part = LidarRanges[part]
                i_min       = int(np.argmin(ranges_part))
                d           = float(ranges_part[i_min])
                ang = angle_min + angle_increment * part[i_min] + theta
                min_dists.append(d)
                min_angles.append(ang)
                xo = x + d * math.cos(ang)
                yo = y + d * math.sin(ang)
                g  = math.hypot(x - xo, y - yo)
                if (g <= gstar) and (g > 1e-6):
                    pr  = kr * ((1/gstar) - (1/g)) * (1/g**3)
                    RF -= pr * np.array([[x - xo], [y - yo]])

        rf_norm  = float(np.linalg.norm(RF))
        F        = AF + RF
        F_norm   = float(np.linalg.norm(F))

        # record graph data
        t_now    = time.time() - self.graph_start
        min_dist = float(np.min(LidarRanges[indices_not_inf])) \
                   if obstacleYES else float('nan')
        with self.graph_lock:
            self.g_time.append(t_now)
            self.g_min_dist.append(min_dist)
            self.g_dist_goal.append(dist_to_goal)
            self.g_af_norm.append(af_norm)
            self.g_rf_norm.append(rf_norm)
            self.g_net_norm.append(F_norm)

        # stuck detection
        self._pos_history.append((x, y))
        if len(self._pos_history) > self.window_size:
            self._pos_history.pop(0)
        travel_w = math.hypot(x - self._pos_history[0][0],
                              y - self._pos_history[0][1]) \
                   if len(self._pos_history) == self.window_size else 1.0

        if self._cooldown_counter > 0:
            self._cooldown_counter -= 1
            robot_stuck = False
        else:
            robot_stuck = (travel_w < self.stuck_dist_threshold) and \
                          (dist_to_goal > self.eps_control)

        if robot_stuck:
            self._stuck_counter += 1
        elif not self._escaping:
            self._stuck_counter = 0

        # goal reached
        if dist_to_goal < self.eps_control:
            print("GOAL REACHED - Waiting for next goal...")
            self.controlVel.linear.x  = 0.0
            self.controlVel.angular.z = 0.0
            self.ControlPublisher.publish(self.controlVel)
            self.has_goal = False
            return

        elif dist_to_goal < self.near_goal_suppress:
            self._stuck_counter = 0
            self._escaping      = False
            thetavel, xvel = self._goal_direct_control(x, y, theta, xd, yd)

        elif self._escaping:
            escape_travel = math.hypot(x - self._escape_start_x,
                                       y - self._escape_start_y)
            if escape_travel > self.escape_min_travel:
                if F_norm < self.f_norm_min_resume:
                    print(f"[ESCAPE EXTEND] F_norm={F_norm:.3f}")
                    eorient  = self.orientationError(theta, self._escape_angle)
                    thetavel = self.kthetap * eorient
                    xvel     = self.escape_speed
                else:
                    print(f"[ESCAPE END] {escape_travel:.3f}m F={F_norm:.3f}")
                    self._escaping         = False
                    self._stuck_counter    = 0
                    self._cooldown_counter = self.post_escape_cooldown
                    self._pos_history.clear()
                    thetavel, xvel = self._normal_control(F, F_norm, theta)
            else:
                eorient = self.orientationError(theta, self._escape_angle)
                thetavel = self.kthetap * eorient
                xvel     = 0.08 if abs(eorient) > abs(self.eps_orient) \
                           else self.escape_speed

        elif self._stuck_counter >= self.stuck_count_threshold:
            if obstacleYES and len(min_dists) > 0:
                obs_ang = min_angles[int(np.argmin(min_dists))]
            else:
                obs_ang = theta
            tang_left  = obs_ang + math.pi/2
            tang_right = obs_ang - math.pi/2
            goal_ang   = math.atan2(yd - y, xd - x)
            diff_l = abs(self.orientationError(tang_left,  goal_ang))
            diff_r = abs(self.orientationError(tang_right, goal_ang))
            if self._last_escape_x is not None and \
               math.hypot(x-self._last_escape_x, y-self._last_escape_y) < 0.5:
                self._escape_flip = not self._escape_flip
            else:
                self._escape_flip = False
            chosen = (tang_right if diff_l < diff_r else tang_left) \
                     if self._escape_flip else \
                     (tang_left  if diff_l < diff_r else tang_right)
            self._escape_angle   = chosen
            self._escape_start_x = x
            self._escape_start_y = y
            self._last_escape_x  = x
            self._last_escape_y  = y
            self._escaping       = True
            self._pos_history.clear()
            print(f"[ESCAPE START] pos=({x:.2f},{y:.2f}) "
                  f"angle={math.degrees(chosen):.1f}°")
            eorient  = self.orientationError(theta, self._escape_angle)
            thetavel = self.kthetap * eorient
            xvel     = 0.0

        else:
            thetavel, xvel = self._normal_control(F, F_norm, theta)

        self.controlVel.linear.x  = xvel
        self.controlVel.linear.y  = 0.0
        self.controlVel.linear.z  = 0.0
        self.controlVel.angular.x = 0.0
        self.controlVel.angular.y = 0.0
        self.controlVel.angular.z = thetavel
        self.ControlPublisher.publish(self.controlVel)

        td = self.msgOdometryTime - self.initialTime
        print(f"Sending the control command\n"
              f"Received pose:\n"
              f"Time,x,y,theta:({td:.3f},{x:.3f},{y:.3f},{theta:.3f})")


# ── Real-time graphs ───────────────────────────────────────────────────────────
# ── Real-time graphs ───────────────────────────────────────────────────────────
def run_graphs(node):
    BG   = '#0f0f23'
    PANE = '#1a1a2e'

    fig, axes = plt.subplots(2, 2, figsize=(13, 7))
    fig.patch.set_facecolor(BG)
    fig.suptitle('Robot Navigation — Real-Time Monitor',
                 fontsize=13, fontweight='bold', color='white')

    ax_dist, ax_goal, ax_forces, ax_net = axes.flatten()

    def style(ax, ylabel, ylim, title):
        ax.set_facecolor(PANE)
        ax.set_ylabel(ylabel, fontsize=9, color='white')
        ax.set_ylim(*ylim)
        ax.set_title(title, fontsize=10, color='#aaaacc', pad=4)
        ax.tick_params(colors='#888899', labelsize=8)
        ax.spines[:].set_color('#333355')
        ax.grid(True, color='#2a2a44', linewidth=0.5)
        ax.set_xlabel('Time (s)', fontsize=9, color='#888899')

    style(ax_dist,   'Distance (m)',  (0, 7),  'Nearest obstacle distance')
    style(ax_goal,   'Distance (m)',  (0, 20), 'Distance to goal')
    style(ax_forces, 'Force (N)',     (0, 12), 'Attractive vs repulsive force')
    style(ax_net,    'Force (N)',     (0, 15), 'Net force magnitude')

    # safety threshold lines
    ax_dist.axhline(0.35, color='#EF9F27', lw=1.2, ls='--',
                    label='Safety 0.35 m', alpha=0.8)
    ax_goal.axhline(0.2,  color='#EF9F27', lw=1.2, ls='--',
                    label='Goal threshold', alpha=0.8)

    line_dist,  = ax_dist.plot(  [], [], color='#E24B4A', lw=1.8,
                                 label='Nearest obstacle')
    line_goal,  = ax_goal.plot(  [], [], color='#2ecc71', lw=1.8,
                                 label='Dist to goal')
    line_af,    = ax_forces.plot([], [], color='#2ecc71', lw=1.8,
                                 label='Attractive |AF|')
    line_rf,    = ax_forces.plot([], [], color='#E24B4A', lw=1.8,
                                 label='Repulsive |RF|')
    line_net,   = ax_net.plot(   [], [], color='#7F77DD', lw=2.0,
                                 label='Net |F|')

    for ax in (ax_dist, ax_goal, ax_forces, ax_net):
        ax.legend(loc='upper right', fontsize=7,
                  facecolor=PANE, edgecolor='#333355',
                  labelcolor='white')

    # live value text boxes
    txt_dist = ax_dist.text(0.02, 0.93, '', transform=ax_dist.transAxes,
                            color='#E24B4A', fontsize=9, va='top')
    txt_goal = ax_goal.text(0.02, 0.93, '', transform=ax_goal.transAxes,
                            color='#2ecc71', fontsize=9, va='top')
    txt_af   = ax_forces.text(0.02, 0.93, '', transform=ax_forces.transAxes,
                              color='#2ecc71', fontsize=9, va='top')
    txt_rf   = ax_forces.text(0.02, 0.80, '', transform=ax_forces.transAxes,
                              color='#E24B4A', fontsize=9, va='top')
    txt_net  = ax_net.text(0.02, 0.93, '', transform=ax_net.transAxes,
                           color='#7F77DD', fontsize=9, va='top')

    def update(_frame):
        with node.graph_lock:
            if len(node.g_time) < 2:
                return
            t   = list(node.g_time)
            md  = list(node.g_min_dist)
            dg  = list(node.g_dist_goal)
            af  = list(node.g_af_norm)
            rf  = list(node.g_rf_norm)
            net = list(node.g_net_norm)

        t_min = max(0, t[-1] - 30)
        t_max = t[-1] + 0.5

        for ax in (ax_dist, ax_goal, ax_forces, ax_net):
            ax.set_xlim(t_min, t_max)

        line_dist.set_data(t, md)
        line_goal.set_data(t, dg)
        line_af.set_data(t, af)
        line_rf.set_data(t, rf)
        line_net.set_data(t, net)

        # colour obstacle line red when dangerously close
        if md and not math.isnan(md[-1]):
            col = '#ff2222' if md[-1] < 0.35 else '#E24B4A'
            line_dist.set_color(col)
            txt_dist.set_text(f'Now: {md[-1]:.2f} m')
            txt_dist.set_color(col)

        if dg:
            txt_goal.set_text(f'Now: {dg[-1]:.2f} m')

        if af and rf:
            txt_af.set_text(f'AF: {af[-1]:.3f} N')
            txt_rf.set_text(f'RF: {rf[-1]:.3f} N')

        if net:
            txt_net.set_text(f'Net: {net[-1]:.3f} N')

        fig.canvas.draw_idle()

    # FIX: assign to fig._ani so Python does not garbage-collect the
    # animation object while the window is open.  Without this reference,
    # FuncAnimation is destroyed immediately after run_graphs() returns and
    # the graph freezes after the first frame.
    fig._ani = animation.FuncAnimation(fig, update, interval=100,
                                       cache_frame_data=False)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()

# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    def signal_handler(sig, frame):
        print("\n[INFO] Force quitting (Ctrl+C)...")
        os._exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)
    node = ControllerNode(
        xdu=10.0, ydu=-10.0,
        kau=0.3,  kru=15.0,
        kthetatu=4.0, gstaru=3.0,
        eps_orientu=np.pi/10,
        eps_controlu=0.2
    )

    ros_thread = threading.Thread(target=rclpy.spin,
                                  args=(node,), daemon=True)
    ros_thread.start()

    try:
        run_graphs(node)
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt received. Closing UI and shutting down...")
    finally:
        import matplotlib.pyplot as plt
        plt.close('all')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        import sys
        sys.exit(0)

if __name__ == '__main__':
    main()