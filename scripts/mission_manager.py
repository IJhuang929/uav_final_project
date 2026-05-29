#!/usr/bin/env python3
"""
mission_manager.py — Phase2-integrated drone FSM.

Architecture based on drone_fsm.py (robotic_2026_final_project_phase2).
Obstacle avoidance upgraded from geometric dodge to ConvexPathOptimizer.

State machine:
  IDLE → TAKEOFF → EXPLORE ⇄ OBSTACLE_REPLAN → APPROACH → LAND

Changes from v1:
  - New OBSTACLE_REPLAN state: ConvexPathOptimizer generates safe waypoints,
    then feeds them to the existing trajectory_planner via /traj/waypoints.
  - drone_fsm.py APPROACH/PRECISION_LAND pattern adapted for Lee controller:
    PRECISION_LAND → enable /apriltag_lander/enable service.
  - /drone_fsm/state published (phase2-compatible) alongside /mission/state.
  - Yaw extracted from odometry quaternion for world-frame obstacle projection.

Topic interface (unchanged for demo_full_mission.sh compatibility):
  Pub  /{mav}/command/pose          PoseStamped         Lee controller (TAKEOFF)
  Pub  /traj/waypoints              nav_msgs/Path        trajectory_planner
  Pub  /traj/cancel                 std_msgs/Empty       trajectory_planner
  Pub  /mission/state               std_msgs/String      state broadcast
  Pub  /drone_fsm/state             std_msgs/String      phase2-compatible state
  Sub  /iris/ground_truth/odometry  nav_msgs/Odometry
  Sub  /obstacle/info               std_msgs/Float32MultiArray
  Sub  /tag_detections              AprilTagDetectionArray
  Sub  /traj/status                 std_msgs/String
  Srv  /apriltag_lander/enable      std_srvs/SetBool
"""
import math
import os
import sys
import threading
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty, Float32MultiArray, String
from std_srvs.srv import SetBool

try:
    from apriltag_ros.msg import AprilTagDetectionArray
    _HAVE_APRILTAG = True
except ImportError:
    _HAVE_APRILTAG = False

# Phase2 ConvexOpt — available when convex_path_optimizer.py is in scripts/
_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

try:
    from convex_path_optimizer import (ConvexPathOptimizer, SphereObstacle,
                                        FlightEnvelope, CVXPY_AVAILABLE)
    _HAVE_CONVEX_OPT = True
except ImportError:
    _HAVE_CONVEX_OPT = False
    CVXPY_AVAILABLE = False

# ── Mission parameters ────────────────────────────────────────────────────────
CRUISE_Z           = 1.5    # m
TAKEOFF_TOL        = 0.15   # m
KI_Z               = 0.5    # altitude integrator gain [1/s]
Z_INT_MAX          = 0.6    # max integral term [m]
TARGET             = (14.0, 0.0, 1.5)
AVOID_TRIGGER_M    = 2.5    # m — trigger OBSTACLE_REPLAN below this distance
DODGE_LATERAL_M    = 2.5    # m — geometric-fallback lateral dodge
DODGE_RESUME_M     = 2.5    # m — geometric-fallback clearance past obstacle
MIN_PASSABLE_W     = 1.0    # m — minimum gap width for gap-steering
REPLAN_COOLDOWN    = 8.0    # s — min interval between replans
CRUISE_SPEED       = 0.8    # m/s
MAV_NAME           = 'iris'

# APPROACH (drone_fsm APPROACH + PRECISION_LAND pattern)
EXPLORE_TAG_TRIGGER  = 3    # consecutive frames to exit EXPLORE → APPROACH
APPROACH_STABLE_N    = 15   # consecutive frames before enabling lander (~1.5 s at 10 Hz)
APPROACH_CLOSE_M     = 4.0  # m — horizontal range within which lander activates
APPROACH_LOST_TICKS  = 20   # ticks (~2 s) without tag before aborting APPROACH

# Phase2 ConvexOpt parameters
DRONE_RADIUS       = 0.25   # m (from drone_fsm / obstacle_avoidance_planner)
OBS_SAFETY_MARGIN  = 0.3    # m
FLIGHT_ENV_KWARGS  = dict(
    x_range=(-1.0, 16.0),
    y_range=(-5.0,  5.0),
    z_range=(0.5,   3.0),
)

# All pillar positions from obstacle_course_v2.world — (cx, cy, radius)
# Registering all of them at once avoids per-replan oscillation between adjacent pillars.
KNOWN_PILLARS = [
    (4.0,   0.0,  0.40),   # P1
    (6.5,   1.8,  0.35),   # P2
    (7.0,  -1.5,  0.35),   # P3
    (9.5,   0.6,  0.40),   # P4
    (11.5, -1.0,  0.30),   # P5
]

# ── State labels ──────────────────────────────────────────────────────────────
IDLE            = 'IDLE'
TAKEOFF         = 'TAKEOFF'
EXPLORE         = 'EXPLORE'
OBSTACLE_REPLAN = 'OBSTACLE_REPLAN'
APPROACH        = 'APPROACH'
LAND            = 'LAND'


class MissionManager:

    def __init__(self):
        rospy.init_node('mission_manager', anonymous=False)

        if not _HAVE_APRILTAG:
            rospy.logwarn('[Mission] apriltag_ros not found — APPROACH disabled')
        if not _HAVE_CONVEX_OPT:
            rospy.logwarn('[Mission] convex_path_optimizer not found — '
                          'OBSTACLE_REPLAN uses geometric fallback only')
        elif not CVXPY_AVAILABLE:
            rospy.logwarn('[Mission] CVXPY not installed — '
                          'ConvexOpt uses greedy fallback (pip install cvxpy)')

        # ── Publishers ────────────────────────────────────────────────────────
        self._pose_pub   = rospy.Publisher(
            f'/{MAV_NAME}/command/pose', PoseStamped, queue_size=1)
        self._traj_pub   = rospy.Publisher(
            '/traj/waypoints', Path, queue_size=1)
        self._cancel_pub = rospy.Publisher(
            '/traj/cancel', Empty, queue_size=1)
        self._state_pub  = rospy.Publisher(
            '/mission/state', String, queue_size=1, latch=True)
        self._fsm_pub    = rospy.Publisher(
            '/drone_fsm/state', String, queue_size=1, latch=True)

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('/iris/ground_truth/odometry', Odometry,
                         self._pose_cb, queue_size=1)
        rospy.Subscriber('/obstacle/info', Float32MultiArray,
                         self._obstacle_cb, queue_size=1)
        rospy.Subscriber('/traj/status', String,
                         self._traj_status_cb, queue_size=1)
        if _HAVE_APRILTAG:
            rospy.Subscriber('/tag_detections', AprilTagDetectionArray,
                             self._tag_cb, queue_size=1)

        # ── Internal state ────────────────────────────────────────────────────
        self._state       = IDLE
        self._entry       = False
        self._pos         = None      # (x, y, z)
        self._yaw         = 0.0       # drone yaw in world frame [rad]
        self._obs         = [0., 1e9, 1e9, 1e9, 0., 0.]  # /obstacle/info
        self._traj_status = 'IDLE'
        self._z_int       = 0.0
        self._last_replan_time    = rospy.Time(0)
        self._replan_thread: threading.Thread | None = None  # async ConvexOpt
        # AprilTag approach state (drone_fsm APPROACH + PRECISION_LAND pattern)
        self._tag_stable_count    = 0     # rolling +1/-1 per frame; replaces bool _tag_seen
        self._tag_world_pos       = None  # (x, y, z) last estimated tag world position
        self._approach_lost_ticks = 0     # ticks without tag while in APPROACH

        rospy.Timer(rospy.Duration(0.1), self._tick)
        rospy.loginfo('[Mission] Waiting for Gazebo pose...')
        rospy.Timer(rospy.Duration(3.0), self._auto_start, oneshot=True)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        # yaw from quaternion (ENU convention)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)

    def _obstacle_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            data = list(msg.data)
            while len(data) < 6:
                data.append(0.0)
            self._obs = data

    def _traj_status_cb(self, msg: String):
        self._traj_status = msg.data

    def _tag_cb(self, msg):
        if msg.detections:
            det = msg.detections[0]
            p = det.pose.pose.pose.position  # camera optical frame
            if self._pos:
                cx, cy, _ = self._pos
                yaw = self._yaw
                # Camera optical: z=forward=body+x, x=right=body-y  (vi_sensor faces +x)
                # body → world: x_w = cx + bx*cos(yaw) - by*sin(yaw)
                #                y_w = cy + bx*sin(yaw) + by*cos(yaw)
                # bx = p.z (forward), by = -p.x (cam right = body -y)
                self._tag_world_pos = (
                    cx + p.z * math.cos(yaw) + p.x * math.sin(yaw),
                    cy + p.z * math.sin(yaw) - p.x * math.cos(yaw),
                    CRUISE_Z,
                )
            self._tag_stable_count = min(self._tag_stable_count + 1, 60)
        else:
            self._tag_stable_count = max(self._tag_stable_count - 1, 0)

    # ── State machine ─────────────────────────────────────────────────────────

    def _auto_start(self, _evt):
        if self._state == IDLE and self._pos is not None:
            rospy.loginfo('[Mission] Pose received — waiting 3 s for RTAB-Map to stabilise...')
            rospy.sleep(3)
            self._go(TAKEOFF)
        elif self._pos is None:
            rospy.logwarn('[Mission] No pose yet — retrying in 2 s')
            rospy.Timer(rospy.Duration(2.0), self._auto_start, oneshot=True)

    def _go(self, new_state: str):
        rospy.loginfo(f'[Mission] {self._state} → {new_state}')
        self._state = new_state
        self._entry = True
        self._state_pub.publish(String(data=new_state))
        self._fsm_pub.publish(String(data=new_state))

    def _tick(self, _evt):
        if self._pos is None:
            return
        {
            TAKEOFF:         self._tick_takeoff,
            EXPLORE:         self._tick_explore,
            OBSTACLE_REPLAN: self._tick_obstacle_replan,
            APPROACH:        self._tick_approach,
            LAND:            self._tick_land,
        }.get(self._state, lambda: None)()

    # ── TAKEOFF ───────────────────────────────────────────────────────────────

    def _tick_takeoff(self):
        if self._entry:
            self._entry = False
            self._z_int = 0.0
            rospy.loginfo(f'[Mission] TAKEOFF → z={CRUISE_Z} m')

        z_err = CRUISE_Z - self._pos[2]
        self._z_int = min(self._z_int + z_err * 0.1, Z_INT_MAX)
        self._send_pose(0.0, 0.0, CRUISE_Z + KI_Z * self._z_int)

        if abs(z_err) < TAKEOFF_TOL:
            rospy.loginfo(f'[Mission] Takeoff done — z_bias={KI_Z * self._z_int:.3f} m')
            self._go(EXPLORE)

    # ── EXPLORE ───────────────────────────────────────────────────────────────

    def _tick_explore(self):
        if self._entry:
            self._entry = False
            self._last_replan_time = rospy.Time(0)
            # Only start fresh trajectory if one isn't already running
            # (entry is False when returning from OBSTACLE_REPLAN)
            if self._traj_status != 'EXECUTING':
                cx, cy, _ = self._pos
                rospy.loginfo(f'[Mission] EXPLORE from ({cx:.1f},{cy:.1f}) → target')
                self._replan_toward_target()
            return

        # P1: AprilTag stably visible → begin active approach (drone_fsm APPROACH pattern)
        if _HAVE_APRILTAG and self._tag_stable_count >= EXPLORE_TAG_TRIGGER:
            self._cancel_pub.publish(Empty())
            self._go(APPROACH)
            return

        # P2: obstacle within alert range → ConvexOpt replan (rate-limited)
        if self._obs[0] > 0.5 and self._obs[1] < AVOID_TRIGGER_M:
            now = rospy.Time.now()
            if (now - self._last_replan_time).to_sec() >= REPLAN_COOLDOWN:
                self._last_replan_time = now
                self._go(OBSTACLE_REPLAN)
            return

        # P3: trajectory finished → resume toward target
        if self._traj_status in ('DONE', 'FAILED'):
            cx, cy, _ = self._pos
            if math.hypot(TARGET[0] - cx, TARGET[1] - cy) > 1.5:
                self._replan_toward_target()
            else:
                rospy.loginfo_throttle(5.0, '[Mission] Near target — hovering')

    # ── OBSTACLE_REPLAN (Phase2 core) ─────────────────────────────────────────

    def _tick_obstacle_replan(self):
        if self._entry:
            self._entry = False
            rospy.loginfo('[Mission] OBSTACLE_REPLAN — cancelling trajectory')
            self._cancel_pub.publish(Empty())
            # Kick off ConvexOpt in a background thread so the 10 Hz timer
            # is never blocked. Hold position while the thread computes.
            self._replan_thread = threading.Thread(
                target=self._replan_worker, daemon=True)
            self._replan_thread.start()
            return

        # Thread still running — hold current position and wait
        if self._replan_thread and self._replan_thread.is_alive():
            if self._pos:
                cx, cy, _ = self._pos
                self._send_pose(cx, cy, CRUISE_Z)
            return

        # Thread finished but state not yet switched (race guard — shouldn't happen)
        rospy.logwarn_throttle(1.0, '[Mission] OBSTACLE_REPLAN: thread done but state not switched')

    def _replan_worker(self):
        """Background thread: run ConvexOpt (slow), then return to EXPLORE."""
        success = self._run_convex_replan() if _HAVE_CONVEX_OPT else False
        if not success:
            rospy.logwarn('[Mission] ConvexOpt unavailable/failed — geometric fallback')
            self._replan_with_avoidance()

        rospy.loginfo('[Mission] OBSTACLE_REPLAN complete → EXPLORE')
        self._state = EXPLORE
        self._entry = False
        self._state_pub.publish(String(data=EXPLORE))
        self._fsm_pub.publish(String(data=EXPLORE))

    # ── APPROACH (drone_fsm APPROACH + PRECISION_LAND pattern) ───────────────
    #
    # Mirrors drone_fsm.py:
    #   APPROACH  — fly toward estimated tag world position; monitor stability
    #   PRECISION_LAND — stable lock (>30 frames) + close range → enable lander
    #                    (PX4 AUTO.PRECLAND → our apriltag_lander equivalent)

    def _tick_approach(self):
        if self._entry:
            self._entry = False
            self._approach_lost_ticks = 0
            rospy.loginfo('[Mission] APPROACH — flying toward tag, awaiting stable lock')
            return

        # ── Tag-loss monitor ──────────────────────────────────────────────────
        if self._tag_stable_count == 0:
            self._approach_lost_ticks += 1
            if self._approach_lost_ticks >= APPROACH_LOST_TICKS:
                rospy.logwarn('[Mission] Tag lost in APPROACH — returning to EXPLORE')
                self._go(EXPLORE)
            return
        self._approach_lost_ticks = 0

        if not self._tag_world_pos:
            return

        tx, ty, _ = self._tag_world_pos
        cx, cy, _ = self._pos
        dist = math.hypot(tx - cx, ty - cy)

        # ── Handoff to lander (drone_fsm PRECISION_LAND equivalent) ──────────
        # When close, forward camera loses the tag (it's below FOV) — that's
        # expected. Hand off to apriltag_lander (downward cam) once we're near
        # the estimated tag position; no stable-count requirement at close range.
        if dist < APPROACH_CLOSE_M:
            rospy.loginfo(
                f'[Mission] Near tag  stable={self._tag_stable_count}'
                f'  dist={dist:.1f}m — enabling apriltag_lander'
            )
            self._enable_lander(True)
            self._go(LAND)
            return

        # ── Fly toward estimated tag world position at cruise altitude ────────
        self._send_pose(tx, ty, CRUISE_Z)
        rospy.loginfo_throttle(1.0,
            f'[Mission] APPROACH  dist={dist:.1f}m'
            f'  stable={self._tag_stable_count}/{APPROACH_STABLE_N}'
        )

    # ── LAND (drone_fsm PRECISION_LAND → apriltag_lander instead of MAVROS) ──

    def _tick_land(self):
        if self._entry:
            self._entry = False
            rospy.loginfo('[Mission] LAND — apriltag_lander in control')

    # ── Phase2 ConvexOpt replan ───────────────────────────────────────────────

    def _run_convex_replan(self) -> bool:
        """
        Registers ALL known pillars in ConvexPathOptimizer and plans a single
        safe path from current position to TARGET.  Using all pillars avoids the
        oscillation that occurs when only one obstacle is registered and the drone
        bounces between adjacent pillars on successive replans.
        Returns True on success.
        """
        cx, cy, _ = self._pos

        optimizer = ConvexPathOptimizer(
            drone_radius=DRONE_RADIUS,
            v_max=CRUISE_SPEED * 2.5,
            a_max=2.0,
            env=FlightEnvelope(**FLIGHT_ENV_KWARGS),
            lambda_smooth=8.0,
            lambda_safe=8.0,
        )

        # Register every pillar so the QP sees the full obstacle field at once.
        for px, py, pr in KNOWN_PILLARS:
            optimizer.add_obstacle(SphereObstacle(
                center=[px, py, CRUISE_Z],
                radius=pr,
                safety_margin=OBS_SAFETY_MARGIN,
            ))

        try:
            safe_wps, info = optimizer.optimize(
                [(cx, cy, CRUISE_Z), TARGET],
                n_intermediate=5,   # more intermediate pts for a 5-pillar field
            )
        except Exception as exc:
            rospy.logwarn(f'[Mission] ConvexOpt exception: {exc}')
            return False

        # Clamp z to cruise altitude — pillars are vertical cylinders, not spheres
        safe_wps = [[float(wp[0]), float(wp[1]), CRUISE_Z] for wp in safe_wps]

        rospy.loginfo(
            f'[Mission] ConvexOpt [{info["solver"]}]  '
            f'pillars={len(KNOWN_PILLARS)}  '
            f'wps={info["n_waypoints"]}  safe={info["path_safe"]}  '
            f'cvxpy={CVXPY_AVAILABLE}'
        )
        if not info['path_safe']:
            rospy.logwarn('[Mission] ConvexOpt path unsafe — geometric fallback')
            return False
        self._send_trajectory(safe_wps, self._path_time(safe_wps))
        return True

    # ── Geometric fallback (unchanged from v1) ────────────────────────────────

    def _replan_toward_target(self):
        cx, cy, _ = self._pos
        wps = [[cx, cy, CRUISE_Z], list(TARGET)]
        self._send_trajectory(wps, self._path_time(wps))
        rospy.loginfo(f'[Mission] Replanning toward target from ({cx:.1f},{cy:.1f})')

    def _replan_with_avoidance(self):
        cx, cy, _ = self._pos
        dist      = self._obs[1]
        left_clr  = self._obs[2]
        right_clr = self._obs[3]
        gap_y     = self._obs[4] if len(self._obs) > 4 else 0.0
        gap_w     = self._obs[5] if len(self._obs) > 5 else 0.0

        if gap_w >= MIN_PASSABLE_W:
            dy     = gap_y
            method = f'gap  gap_y={gap_y:+.2f}m  gap_w={gap_w:.2f}m'
        else:
            dy     = +DODGE_LATERAL_M if left_clr > right_clr else -DODGE_LATERAL_M
            method = f'fallback {"L" if dy > 0 else "R"}  gap_w={gap_w:.2f}m'

        x_sidestep = cx + max(dist - 1.0, 0.5)
        x_clear    = max(cx + dist + DODGE_RESUME_M, x_sidestep + 1.0)

        wps = [
            [cx,         cy,      CRUISE_Z],
            [x_sidestep, cy + dy, CRUISE_Z],
            [x_clear,    cy + dy, CRUISE_Z],
            list(TARGET),
        ]
        self._send_trajectory(wps, self._path_time(wps))
        rospy.loginfo(
            f'[Mission] Geometric dodge [{method}]  '
            f'obs={dist:.2f}m  sidestep=({x_sidestep:.1f},{cy+dy:.1f})'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _send_pose(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._pose_pub.publish(msg)

    def _send_trajectory(self, waypoints: list, total_time: float):
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp    = rospy.Time(total_time)
        z_bias = KI_Z * self._z_int
        for wp in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            arr = np.array(wp, dtype=float)
            ps.pose.position.x = float(arr[0])
            ps.pose.position.y = float(arr[1])
            ps.pose.position.z = float(arr[2]) + z_bias
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._traj_pub.publish(path)

    def _path_time(self, waypoints: list) -> float:
        pts = np.array([[w[0], w[1], w[2]] for w in waypoints], dtype=float)
        total_dist = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
        return max(total_dist / CRUISE_SPEED, 5.0)

    def _enable_lander(self, enable: bool):
        try:
            rospy.wait_for_service('/apriltag_lander/enable', timeout=2.0)
            srv = rospy.ServiceProxy('/apriltag_lander/enable', SetBool)
            srv(enable)
            rospy.loginfo(f'[Mission] lander enable={enable}')
        except Exception as exc:
            rospy.logwarn(f'[Mission] /apriltag_lander/enable unavailable: {exc}')

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        MissionManager().run()
    except rospy.ROSInterruptException:
        pass
