#!/usr/bin/env python3
"""
Potential field mobile robot controller — ROS2 / Gazebo Harmonic
with real-time obstacle distance plot.

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
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

import matplotlib
matplotlib.use('TkAgg')          # works in WSL with an X server / WSLg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

topic1 = '/cmd_vel'
topic2 = '/odom1'
topic3 = '/scan'

# ── Graph history length (number of data points shown) ────────────────────────
HISTORY = 200   # ~10 seconds at 20 Hz


class ControllerNode(Node):

    def __init__(self, xdu, ydu, kau, kru, kthetatu, gstaru,
                 eps_orientu, eps_controlu):
        super().__init__('controller_node')

        # Goal & gains
        self.xdp         = xdu
        self.ydp         = ydu
        self.kap         = kau
        self.krp         = kru
        self.kthetap     = kthetatu
        self.gstarp      = gstaru
        self.eps_orient  = eps_orientu
        self.eps_control = eps_controlu

        # ── Stuck / escape parameters ──────────────────────────────────────
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

        # ── Frozen sim detection ───────────────────────────────────────────
        self._last_sim_time      = None
        self._last_sim_wall_time = time.time()
        self._sim_frozen         = False

        # ── Real-time graph data (thread-safe deques) ──────────────────────
        self.graph_lock       = threading.Lock()
        self.graph_times      = deque(maxlen=HISTORY)   # wall-clock seconds
        self.graph_min_dist   = deque(maxlen=HISTORY)   # nearest obstacle (m)
        self.graph_dist_goal  = deque(maxlen=HISTORY)   # distance to goal (m)
        self.graph_start_wall = time.time()

        # ── ROS messages ───────────────────────────────────────────────────
        self.OdometryMsg     = Odometry()
        self.LidarMsg        = LaserScan()
        self.initialTime     = time.time()
        self.msgOdometryTime = time.time()
        self.msgLidarTime    = time.time()

        self.controlVel = Twist()

        self.ControlPublisher = self.create_publisher(Twist, topic1, 10)
        self.PoseSubscriber   = self.create_subscription(
            Odometry,  topic2, self.SensorCallbackPose,  10)
        self.LidarSubscriber  = self.create_subscription(
            LaserScan, topic3, self.SensorCallbackLidar, 10)

        self.period = 0.05
        self.timer  = self.create_timer(self.period, self.ControlFunction)

    # ── Callbacks ──────────────────────────────────────────────────────────
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

    # ── Orientation error ──────────────────────────────────────────────────
    def orientationError(self, theta_, thetad_):
        if (thetad_ > np.pi/2) and (thetad_ <= np.pi):
            if (theta_ > -np.pi) and (theta_ <= -np.pi/2):
                theta_ += 2*np.pi
        if (theta_ > np.pi/2) and (theta_ <= np.pi):
            if (thetad_ > -np.pi) and (thetad_ <= -np.pi/2):
                thetad_ += 2*np.pi
        return thetad_ - theta_

    # ── Normal PF control ──────────────────────────────────────────────────
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

    # ── Direct goal approach ───────────────────────────────────────────────
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

    # ── Main control loop ──────────────────────────────────────────────────
    def ControlFunction(self):
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

        # ── Frozen sim check ───────────────────────────────────────────────
        wall_since_update = time.time() - self._last_sim_wall_time
        if wall_since_update > 2.0:
            if not self._sim_frozen:
                print(f"[SIM FROZEN] stopping motors.")
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

        # ── Attractive force ───────────────────────────────────────────────
        vectorD      = np.array([[x - xd], [y - yd]])
        AF           = -ka * vectorD
        dist_to_goal = float(np.linalg.norm(vectorD))

        # ── Repulsive force ────────────────────────────────────────────────
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

        # ── Total force ────────────────────────────────────────────────────
        F      = AF + RF
        F_norm = float(np.linalg.norm(F))

        # ── Update graph data ──────────────────────────────────────────────
        t_now    = time.time() - self.graph_start_wall
        min_dist = float(np.min(LidarRanges[indices_not_inf])) \
                   if obstacleYES else float('nan')
        with self.graph_lock:
            self.graph_times.append(t_now)
            self.graph_min_dist.append(min_dist)
            self.graph_dist_goal.append(dist_to_goal)

        # ── Window-based stuck detection ───────────────────────────────────
        self._pos_history.append((x, y))
        if len(self._pos_history) > self.window_size:
            self._pos_history.pop(0)

        if len(self._pos_history) == self.window_size:
            ox, oy   = self._pos_history[0]
            travel_w = math.hypot(x - ox, y - oy)
        else:
            travel_w = 1.0

        if self._cooldown_counter > 0:
            self._cooldown_counter -= 1
            robot_stuck = False
        else:
            robot_stuck = (travel_w < self.stuck_dist_threshold) and \
                          (dist_to_goal > self.eps_control)

        if robot_stuck:
            self._stuck_counter += 1
        else:
            if not self._escaping:
                self._stuck_counter = 0

        # ── Goal reached ───────────────────────────────────────────────────
        if dist_to_goal < self.eps_control:
            print("GOAL REACHED")
            self.controlVel.linear.x  = 0.0
            self.controlVel.angular.z = 0.0
            self.ControlPublisher.publish(self.controlVel)
            rclpy.shutdown()
            return

        # ── Near goal — direct drive ───────────────────────────────────────
        elif dist_to_goal < self.near_goal_suppress:
            self._stuck_counter = 0
            self._escaping      = False
            thetavel, xvel = self._goal_direct_control(x, y, theta, xd, yd)

        # ── Escape mode ────────────────────────────────────────────────────
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
                    print(f"[ESCAPE END] travelled {escape_travel:.3f} m  "
                          f"F_norm={F_norm:.3f}")
                    self._escaping         = False
                    self._stuck_counter    = 0
                    self._cooldown_counter = self.post_escape_cooldown
                    self._pos_history.clear()
                    thetavel, xvel = self._normal_control(F, F_norm, theta)
            else:
                eorient = self.orientationError(theta, self._escape_angle)
                if abs(eorient) > abs(self.eps_orient):
                    thetavel = self.kthetap * eorient
                    xvel     = 0.08
                else:
                    thetavel = self.kthetap * eorient
                    xvel     = self.escape_speed

        # ── Trigger escape ─────────────────────────────────────────────────
        elif self._stuck_counter >= self.stuck_count_threshold:
            if obstacleYES and len(min_dists) > 0:
                nearest = int(np.argmin(min_dists))
                obs_ang = min_angles[nearest]
            else:
                obs_ang = theta

            tang_left  = obs_ang + math.pi/2
            tang_right = obs_ang - math.pi/2
            goal_ang   = math.atan2(yd - y, xd - x)
            diff_l = abs(self.orientationError(tang_left,  goal_ang))
            diff_r = abs(self.orientationError(tang_right, goal_ang))

            if self._last_escape_x is not None:
                dist_from_last = math.hypot(x - self._last_escape_x,
                                            y - self._last_escape_y)
                if dist_from_last < 0.5:
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

            print(f"[ESCAPE START] stuck={self._stuck_counter} "
                  f"pos=({x:.2f},{y:.2f}) "
                  f"angle={math.degrees(chosen):.1f}° "
                  f"F_norm={F_norm:.3f}")

            eorient  = self.orientationError(theta, self._escape_angle)
            thetavel = self.kthetap * eorient
            xvel     = 0.0

        # ── Normal PF ──────────────────────────────────────────────────────
        else:
            thetavel, xvel = self._normal_control(F, F_norm, theta)

        # ── Publish ────────────────────────────────────────────────────────
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


# ── Real-time graph ────────────────────────────────────────────────────────────
def run_graph(node):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    fig.suptitle('Robot Navigation — Real-Time Monitor', fontsize=13,
                 fontweight='bold')

    line_dist,  = ax1.plot([], [], color='#e74c3c', linewidth=1.8,
                           label='Nearest obstacle (m)')
    ax1.axhline(y=0.35, color='orange', linestyle='--', linewidth=1.2,
                label='Safety threshold (0.35 m)')
    ax1.set_ylabel('Distance to\nnearest obstacle (m)', fontsize=10)
    ax1.set_ylim(0, 6)
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_facecolor('#1a1a2e')
    line_dist.set_color('#e74c3c')

    line_goal,  = ax2.plot([], [], color='#2ecc71', linewidth=1.8,
                           label='Distance to goal (m)')
    ax2.axhline(y=0.2, color='#f39c12', linestyle='--', linewidth=1.2,
                label='Goal threshold (0.2 m)')
    ax2.set_ylabel('Distance to\ngoal (m)', fontsize=10)
    ax2.set_xlabel('Time (s)', fontsize=10)
    ax2.set_ylim(0, 20)
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_facecolor('#1a1a2e')

    fig.patch.set_facecolor('#0f0f23')
    for ax in (ax1, ax2):
        ax.tick_params(colors='white')
        ax.yaxis.label.set_color('white')
        ax.xaxis.label.set_color('white')
        ax.spines[:].set_color('#444')
    fig.suptitle('Robot Navigation — Real-Time Monitor',
                 fontsize=13, fontweight='bold', color='white')

    def update(_frame):
        with node.graph_lock:
            if len(node.graph_times) < 2:
                return line_dist, line_goal
            t   = list(node.graph_times)
            md  = list(node.graph_min_dist)
            dg  = list(node.graph_dist_goal)

        line_dist.set_data(t, md)
        line_goal.set_data(t, dg)

        t_min = t[-1] - 30 if t[-1] > 30 else 0
        ax1.set_xlim(t_min, t[-1] + 0.5)
        ax2.set_xlim(t_min, t[-1] + 0.5)

        # colour the obstacle line red when too close
        if md and not math.isnan(md[-1]):
            line_dist.set_color('#e74c3c' if md[-1] > 0.35 else '#ff0000')

        return line_dist, line_goal

    ani = animation.FuncAnimation(fig, update, interval=100,
                                  blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode(
        xdu=10.0, ydu=-10.0,
        kau=0.3,  kru=15.0,
        kthetatu=4.0, gstaru=3.0,
        eps_orientu=np.pi/10,
        eps_controlu=0.2
    )

    # Run ROS2 spin in a background thread so matplotlib can own the main thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Real-time graph runs on main thread (required by matplotlib/Tk)
    try:
        run_graph(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()