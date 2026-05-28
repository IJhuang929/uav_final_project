#!/usr/bin/env python3
"""
apriltag_lander.py  ── AprilTag Precision Landing Node
-------------------------------------------------------
All control goes through PoseStamped → /iris/command/pose
(Lee Position Controller).  /iris/command/velocity has no subscriber
in the RotorS default launch and is NOT used here.

vi_sensor camera optical frame (faces forward, +body-x direction):
  camera z = body +x  (depth = forward distance to tag)
  camera x = body -y  (image-right = drone-right; ENU y = left)
  camera y ≈ drone altitude above tag (down in image = lower in world)

State machine:
  TAKEOFF    → climb to takeoff_alt with publish_pose
  HOVER      → hold position, stabilise for hover_sec
  SEARCHING  → hold position, wait for tag
  CENTERING  → correct lateral (camera-x → body-y) until |ex| < thr
  DESCENDING → forward approach (ez→vx) + lateral hold + descent
               until altitude (from odometry) < land_z
  LANDED     → mission complete

Service /apriltag_lander/enable (std_srvs/SetBool):
  True  → start from TAKEOFF, initialise cmd setpoint from odometry
  False → hold position
"""
import os
import rospy
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse

TAKEOFF    = "TAKEOFF"
HOVER      = "HOVER"
SEARCHING  = "SEARCHING"
CENTERING  = "CENTERING"
DESCENDING = "DESCENDING"
LANDED     = "LANDED"


class AprilTagLander:
    def __init__(self):
        rospy.init_node('apriltag_lander', anonymous=False)

        # ── Parameters ─────────────────────────────────────────────────────────
        self.tag_id      = rospy.get_param('~tag_id', 0)
        self.kp_xy       = rospy.get_param('~kp_xy', 0.5)
        self.kp_approach = rospy.get_param('~kp_approach', 0.15)  # forward gain
        self.descent_vel = rospy.get_param('~descent_vel', -0.15)  # m/s (negative)
        self.center_thr  = rospy.get_param('~center_threshold', 0.08)  # m
        self.approach_stop_ez = rospy.get_param('~approach_stop_ez', 0.20)  # m
        self.land_z      = rospy.get_param('~land_z', 0.25)  # m altitude → LANDED
        self.max_vxy     = rospy.get_param('~max_vel_xy', 0.6)
        self.ns          = rospy.get_param('~mav_name', 'iris')
        self.takeoff_alt = rospy.get_param('~takeoff_alt', 1.5)  # m
        self.takeoff_thr = rospy.get_param('~takeoff_threshold', 0.10)  # m
        self.hover_sec   = rospy.get_param('~hover_seconds', 2.0)

        # ── Publisher (Lee Position Controller) ────────────────────────────────
        self.pose_pub = rospy.Publisher(
            f'/{self.ns}/command/pose', PoseStamped, queue_size=1)

        # ── Subscribers ────────────────────────────────────────────────────────
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_cb)
        rospy.Subscriber('/iris/ground_truth/odometry', Odometry, self.odom_cb)

        # ── State ──────────────────────────────────────────────────────────────
        self.state           = TAKEOFF
        self.tag_pos         = None
        self.tag_seen        = False
        self.center_ok_count = 0
        self._lost_count     = 0
        self._hover_start    = None
        self.rate            = rospy.Rate(20)

        # Drone position from odometry
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.0

        # Running position setpoint used in CENTERING / DESCENDING
        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = 0.0

        # Standalone demo (demo_landing.sh): starts enabled.
        # Full mission (demo_full_mission.sh): pass LANDER_INITIALLY_ENABLED=false
        # so mission_manager controls when landing begins.
        _env = os.environ.get('LANDER_INITIALLY_ENABLED', 'true').lower()
        self._enabled = _env not in ('false', '0', 'no')
        rospy.Service('/apriltag_lander/enable', SetBool, self._enable_cb)

        rospy.loginfo(f'[Lander] Init — mav={self.ns}  takeoff_alt={self.takeoff_alt}m')
        rospy.loginfo('[Lander] Waiting for first odometry message...')
        while not rospy.is_shutdown() and self.cur_z == 0.0:
            self.rate.sleep()
        rospy.loginfo(f'[Lander] Odom ready — z={self.cur_z:.2f}m')

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_z = msg.pose.pose.position.z

    def tag_cb(self, msg: AprilTagDetectionArray):
        for det in msg.detections:
            if self.tag_id in det.id:
                p = det.pose.pose.pose.position
                self.tag_pos  = np.array([p.x, p.y, p.z])
                self.tag_seen = True
                return
        self.tag_seen = False

    def _enable_cb(self, req: SetBool):
        self._enabled = req.data
        if req.data:
            self._cmd_x = self.cur_x
            self._cmd_y = self.cur_y
            self._cmd_z = self.cur_z
            self.state           = TAKEOFF
            self.tag_seen        = False
            self.center_ok_count = 0
            self._lost_count     = 0
            self._hover_start    = None
            rospy.loginfo('[Lander] Enabled — restarting TAKEOFF')
        else:
            rospy.loginfo('[Lander] Disabled — holding position')
        return SetBoolResponse(success=True, message='ok')

    # ── Helpers ────────────────────────────────────────────────────────────────
    def clamp(self, v, limit):
        return float(np.clip(v, -limit, limit))

    def publish_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='world')
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def update_cmd(self, vx=0., vy=0., vz=0.):
        """Integrate body-frame velocity into running world-frame setpoint."""
        dt = 1.0 / 20
        self._cmd_x += vx * dt
        self._cmd_y += vy * dt
        self._cmd_z += vz * dt
        self.publish_pose(self._cmd_x, self._cmd_y, self._cmd_z)

    def hold(self):
        """Publish current setpoint without moving."""
        self.publish_pose(self._cmd_x, self._cmd_y, self._cmd_z)

    def transition(self, new_state):
        rospy.loginfo(f'[Lander] {self.state} → {new_state}')
        self.state           = new_state
        self.center_ok_count = 0

    # ── Main loop ──────────────────────────────────────────────────────────────
    def run(self):
        while not rospy.is_shutdown():

            if not self._enabled:
                self.rate.sleep()
                continue

            # ── TAKEOFF ───────────────────────────────────────────────────────
            if self.state == TAKEOFF:
                self.publish_pose(self.cur_x, self.cur_y, self.takeoff_alt)
                err = self.takeoff_alt - self.cur_z
                rospy.loginfo_throttle(1,
                    f'[TAKEOFF] z={self.cur_z:.2f}m → {self.takeoff_alt}m  err={err:.2f}m')
                if abs(err) < self.takeoff_thr:
                    self._hover_start = rospy.Time.now()
                    self.transition(HOVER)

            # ── HOVER: hold position, let controller settle ───────────────────
            elif self.state == HOVER:
                self.publish_pose(self.cur_x, self.cur_y, self.takeoff_alt)
                elapsed = (rospy.Time.now() - self._hover_start).to_sec()
                rospy.loginfo_throttle(1,
                    f'[HOVER] stable {elapsed:.1f}/{self.hover_sec}s')
                if elapsed >= self.hover_sec:
                    # Initialise running cmd setpoint from current position
                    self._cmd_x = self.cur_x
                    self._cmd_y = self.cur_y
                    self._cmd_z = self.cur_z
                    self.transition(SEARCHING)

            # ── SEARCHING: hold, wait for tag ─────────────────────────────────
            elif self.state == SEARCHING:
                self.hold()
                rospy.loginfo_throttle(2, '[SEARCHING] waiting for AprilTag...')
                if self.tag_seen:
                    self.transition(CENTERING)

            # ── CENTERING: lateral alignment (camera-x → body-y) ─────────────
            elif self.state == CENTERING:
                if not self.tag_seen:
                    self._lost_count += 1
                    if self._lost_count > 60:
                        rospy.logwarn_throttle(3, '[Lander] Tag lost — holding')
                    self.hold()
                    self.rate.sleep()
                    continue
                self._lost_count = 0

                ex, ez = self.tag_pos[0], self.tag_pos[2]
                # camera x (right) = body -y (ENU y = left)
                vy = self.clamp(-self.kp_xy * ex, self.max_vxy)

                rospy.loginfo_throttle(1,
                    f'[CENTERING] ex={ex:.3f}m ez={ez:.2f}m vy_cmd={vy:.2f}')
                self.update_cmd(vx=0., vy=vy, vz=0.)

                if abs(ex) < self.center_thr:
                    self.center_ok_count += 1
                    if self.center_ok_count > 10:   # stable 0.5 s
                        self.transition(DESCENDING)
                else:
                    self.center_ok_count = 0

            # ── DESCENDING: forward approach + lateral hold + descent ─────────
            elif self.state == DESCENDING:
                if not self.tag_seen:
                    self._lost_count += 1
                    if self._lost_count > 40:
                        rospy.logwarn('[Lander] Tag lost during descent')
                        self.transition(CENTERING)
                    self.hold()
                    self.rate.sleep()
                    continue
                self._lost_count = 0

                ex, ez = self.tag_pos[0], self.tag_pos[2]
                vy = self.clamp(-self.kp_xy * ex * 0.6, self.max_vxy * 0.5)

                # Forward approach: camera z (= body +x) closes the distance.
                # Stop advancing when within approach_stop_ez of the tag.
                vx = self.clamp(
                    self.kp_approach * max(ez - self.approach_stop_ez, 0.0),
                    self.max_vxy * 0.5)
                vz = self.descent_vel

                if abs(ex) > self.center_thr * 2.5:
                    self.transition(CENTERING)
                    self.rate.sleep()
                    continue

                rospy.loginfo_throttle(0.5,
                    f'[DESCENDING] ez={ez:.2f}m ex={ex:.3f}m alt={self.cur_z:.2f}m '
                    f'cmd=({vx:.2f},{vy:.2f},{vz:.2f})')
                self.update_cmd(vx=vx, vy=vy, vz=vz)

                # Land when altitude drops below threshold
                if self.cur_z < self.land_z:
                    rospy.loginfo(f'[Lander] Landed!  alt={self.cur_z:.2f}m  ez={ez:.2f}m')
                    self.transition(LANDED)

            # ── LANDED ────────────────────────────────────────────────────────
            elif self.state == LANDED:
                self.hold()
                rospy.loginfo_once('[Lander] Mission complete.')
                break

            self.rate.sleep()

        self.hold()
        rospy.loginfo('[Lander] Done.')


if __name__ == '__main__':
    try:
        AprilTagLander().run()
    except rospy.ROSInterruptException:
        pass
