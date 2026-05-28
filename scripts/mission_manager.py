#!/usr/bin/env python3
"""
mission_manager.py — Obstacle-avoidance + AprilTag landing mission.

State machine:
  IDLE → TAKEOFF → EXPLORE → ALIGN → LAND
  Avoidance is handled inside EXPLORE via continuous trajectory replanning:
    obstacle detected → _replan_with_avoidance() → trajectory_planner
    trajectory DONE   → _replan_toward_target()  → trajectory_planner

Topic interface:
  Pub  /{mav}/command/pose   PoseStamped         Lee controller (TAKEOFF)
  Pub  /traj/waypoints       nav_msgs/Path        trajectory_planner
  Pub  /traj/cancel          std_msgs/Empty       trajectory_planner
  Sub  /iris/ground_truth/odometry  Odometry     drone position (gazebo_controller)
  Sub  /obstacle/info        Float32MultiArray    obstacle_detector
  Sub  /tag_detections       AprilTagDetectionArray  apriltag_ros
  Sub  /traj/status          std_msgs/String      trajectory_planner
  Srv  /apriltag_lander/enable  std_srvs/SetBool  apriltag_lander

/obstacle/info layout:  data[0]=detected  data[1]=dist(m)
                        data[2]=left_clear  data[3]=right_clear
/traj/waypoints:        header.stamp.to_sec() encodes total_time (s)
"""
import math
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

# ── Mission parameters ────────────────────────────────────────────────────────
CRUISE_Z           = 1.5    # m — cruise altitude
TAKEOFF_TOL        = 0.15   # m — altitude band to declare takeoff complete
KI_Z               = 0.5    # integrator gain for altitude steady-state correction [1/s]
Z_INT_MAX          = 0.6    # max integral term [m] — caps cmd_z overshoot
TARGET             = (14.0, 0.0, 1.5)   # forward waypoint
AVOID_TRIGGER_M    = 3.5    # m — start avoidance when obstacle closer than this
DODGE_LATERAL_M    = 2.5    # m — fallback lateral dodge when no gap detected
DODGE_RESUME_M     = 2.5    # m — clearance past obstacle before resuming original track
MIN_PASSABLE_W     = 1.0    # m — minimum detected gap width to fly through
REPLAN_COOLDOWN    = 3.0    # s — min seconds between avoidance replans
CRUISE_SPEED       = 0.8    # m/s — used for dynamic trajectory time allocation
MAV_NAME           = 'iris'

# ── State labels ──────────────────────────────────────────────────────────────
IDLE    = 'IDLE'
TAKEOFF = 'TAKEOFF'
EXPLORE = 'EXPLORE'
ALIGN   = 'ALIGN'
LAND    = 'LAND'


class MissionManager:

    def __init__(self):
        rospy.init_node('mission_manager', anonymous=False)

        if not _HAVE_APRILTAG:
            rospy.logwarn('[Mission] apriltag_ros not found — ALIGN state disabled')

        # ── Publishers ────────────────────────────────────────────────────────
        self._pose_pub   = rospy.Publisher(
            f'/{MAV_NAME}/command/pose', PoseStamped, queue_size=1)
        self._traj_pub   = rospy.Publisher(
            '/traj/waypoints', Path, queue_size=1)
        self._cancel_pub = rospy.Publisher(
            '/traj/cancel', Empty, queue_size=1)
        self._state_pub  = rospy.Publisher(
            '/mission/state', String, queue_size=1, latch=True)

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
        self._entry       = False          # True on first tick after transition
        self._pos         = None           # (x, y, z) drone position
        self._obs         = [0., 1e9, 1e9, 1e9]   # [detected, dist, L, R]
        self._tag_seen    = False
        self._traj_status = 'IDLE'
        self._z_int           = 0.0        # altitude error integral for gravity-bias correction
        self._last_replan_time = rospy.Time(0)  # tracks avoidance replan cooldown

        # ── State machine at 10 Hz ────────────────────────────────────────────
        rospy.Timer(rospy.Duration(0.1), self._tick)

        rospy.loginfo('[Mission] Waiting for Gazebo pose...')
        rospy.Timer(rospy.Duration(3.0), self._auto_start, oneshot=True)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos = (p.x, p.y, p.z)

    def _obstacle_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self._obs = list(msg.data)

    def _traj_status_cb(self, msg: String):
        self._traj_status = msg.data

    def _tag_cb(self, msg):
        self._tag_seen = len(msg.detections) > 0

    # ── State machine ─────────────────────────────────────────────────────────

    def _auto_start(self, _evt):
        if self._state == IDLE and self._pos is not None:
            rospy.loginfo('[Mission] Pose received — waiting 3 s for RTAB-Map to stabilise...')
            rospy.sleep(3)
            self._go(TAKEOFF)
        elif self._pos is None:
            rospy.logwarn('[Mission] No pose yet — retrying start in 2 s')
            rospy.Timer(rospy.Duration(2.0), self._auto_start, oneshot=True)

    def _go(self, new_state: str):
        rospy.loginfo(f'[Mission] {self._state} → {new_state}')
        self._state = new_state
        self._entry = True
        self._state_pub.publish(String(data=new_state))

    def _tick(self, _evt):
        if self._pos is None:
            return
        {
            TAKEOFF: self._tick_takeoff,
            EXPLORE: self._tick_explore,
            ALIGN:   self._tick_align,
            LAND:    self._tick_land,
        }.get(self._state, lambda: None)()

    # ── TAKEOFF ───────────────────────────────────────────────────────────────

    def _tick_takeoff(self):
        if self._entry:
            self._entry = False
            self._z_int = 0.0
            rospy.loginfo(f'[Mission] TAKEOFF → z={CRUISE_Z} m')

        z_err = CRUISE_Z - self._pos[2]
        self._z_int = min(self._z_int + z_err * 0.1, Z_INT_MAX)  # dt=0.1 s @ 10 Hz
        self._send_pose(0.0, 0.0, CRUISE_Z + KI_Z * self._z_int)

        if abs(z_err) < TAKEOFF_TOL:
            rospy.loginfo(f'[Mission] Takeoff done — z_bias={KI_Z * self._z_int:.3f} m')
            self._go(EXPLORE)

    # ── EXPLORE ───────────────────────────────────────────────────────────────

    def _tick_explore(self):
        if self._entry:
            self._entry = False
            self._last_replan_time = rospy.Time(0)
            cx, cy, _ = self._pos
            rospy.loginfo(f'[Mission] EXPLORE from ({cx:.1f},{cy:.1f}) → target')
            self._replan_toward_target()
            return

        # Priority 1: AprilTag in sight → hand over to lander
        if _HAVE_APRILTAG and self._tag_seen:
            self._cancel_pub.publish(Empty())
            self._go(ALIGN)
            return

        # Priority 2: obstacle within alert range → replan with avoidance (rate-limited)
        if self._obs[0] > 0.5 and self._obs[1] < AVOID_TRIGGER_M:
            now = rospy.Time.now()
            if (now - self._last_replan_time).to_sec() >= REPLAN_COOLDOWN:
                self._last_replan_time = now
                self._replan_with_avoidance()
            return

        # Priority 3: trajectory finished → resume toward target or hover if arrived
        if self._traj_status == 'DONE':
            cx, cy, _ = self._pos
            if math.hypot(TARGET[0] - cx, TARGET[1] - cy) > 1.5:
                self._replan_toward_target()
            else:
                rospy.loginfo_throttle(5.0, '[Mission] Near target — hovering')

    # ── ALIGN ─────────────────────────────────────────────────────────────────

    def _tick_align(self):
        if self._entry:
            self._entry = False
            rospy.loginfo('[Mission] ALIGN → stopping planner, enabling lander')
            self._cancel_pub.publish(Empty())
            rospy.sleep(0.1)   # ensure cancel propagates before lander takes control
            self._enable_lander(True)
            self._go(LAND)

    # ── LAND ──────────────────────────────────────────────────────────────────

    def _tick_land(self):
        if self._entry:
            self._entry = False
            rospy.loginfo('[Mission] LAND — apriltag_lander in control')

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
        path.header.stamp    = rospy.Time(total_time)  # encode total_time
        z_bias = KI_Z * self._z_int        # apply learned gravity-bias correction
        for wp in waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.position.z = float(wp[2]) + z_bias
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._traj_pub.publish(path)

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
            # Steer toward the sensor-detected gap centre
            dy     = gap_y
            method = f'gap  gap_y={gap_y:+.2f}m  gap_w={gap_w:.2f}m'
        else:
            # No clear gap found — fall back to whichever side is more open
            dy     = +DODGE_LATERAL_M if left_clr > right_clr else -DODGE_LATERAL_M
            method = f'fallback {"L" if dy > 0 else "R"}  gap_w={gap_w:.2f}m (too narrow)'

        # sidestep: begin lateral movement 1 m before the obstacle
        x_sidestep = cx + max(dist - 1.0, 0.5)
        # clearance: DODGE_RESUME_M past the obstacle, still offset laterally
        x_clear    = max(cx + dist + DODGE_RESUME_M, x_sidestep + 1.0)

        wps = [
            [cx,         cy,      CRUISE_Z],   # current position
            [x_sidestep, cy + dy, CRUISE_Z],   # begin lateral movement
            [x_clear,    cy + dy, CRUISE_Z],   # fully past obstacle
            list(TARGET),                       # minimum-snap returns to track
        ]
        self._send_trajectory(wps, self._path_time(wps))
        rospy.loginfo(
            f'[Mission] Avoidance replan [{method}]  '
            f'obs={dist:.2f}m  sidestep=({x_sidestep:.1f},{cy+dy:.1f})'
        )

    def _path_time(self, waypoints: list) -> float:
        pts = np.array([[wp[0], wp[1], wp[2]] for wp in waypoints], dtype=float)
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
