#!/usr/bin/env python3
"""
apriltag_lander.py  ── AprilTag Precision Landing Node (downward camera)
------------------------------------------------------------------------
Subscribes to /down_cam/tag_detections from the downward-facing camera
(/iris/camera_down/image_raw).  The forward vi_sensor is NOT touched here;
it continues feeding obstacle_detector and the forward apriltag_ros instance
that mission_manager uses for EXPLORE → APPROACH.

Downward camera coordinate mapping (camera rpy="0 pi/2 0" on base_link):
  camera_link +X  = body −Z  (optical axis points downward)
  optical Z  = altitude above tag  (p.z = distance to ground)
  optical X  = body −Y direction  →  vy = −kp × p.x
  optical Y  = body −X direction  →  vx = −kp × p.y

State machine:
  TAKEOFF    → climb to takeoff_alt
  HOVER      → hold and stabilise for hover_sec
  SEARCHING  → hold, wait for tag in downward camera
  CENTERING  → correct lateral (p.x → vy, p.y → vx) until centred
  DESCENDING → maintain centering + descend until altitude < land_z
  LANDED     → done

Service /apriltag_lander/enable (std_srvs/SetBool):
  True  → restart from TAKEOFF, init setpoint from current odometry
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
        self.descent_vel = rospy.get_param('~descent_vel', -0.15)    # m/s (negative)
        self.center_thr  = rospy.get_param('~center_threshold', 0.08) # m
        self.land_z      = rospy.get_param('~land_z', 0.25)           # m altitude
        self.max_vxy     = rospy.get_param('~max_vel_xy', 0.6)
        self.ns          = rospy.get_param('~mav_name', 'iris')
        self.takeoff_alt = rospy.get_param('~takeoff_alt', 1.5)
        self.takeoff_thr = rospy.get_param('~takeoff_threshold', 0.10)
        self.hover_sec   = rospy.get_param('~hover_seconds', 2.0)
        # ROS namespace where downward apriltag_ros publishes
        self.down_ns     = rospy.get_param('~down_cam_ns', 'down_cam')

        # ── Publisher (Lee Position Controller) ────────────────────────────────
        self.pose_pub = rospy.Publisher(
            f'/{self.ns}/command/pose', PoseStamped, queue_size=1)

        # ── Subscribers ────────────────────────────────────────────────────────
        down_topic = f'/{self.down_ns}/tag_detections'
        rospy.Subscriber(down_topic, AprilTagDetectionArray, self.tag_cb)
        rospy.loginfo(f'[Lander] Downward tag topic: {down_topic}')

        rospy.Subscriber('/iris/ground_truth/odometry', Odometry, self.odom_cb)

        # ── State ──────────────────────────────────────────────────────────────
        self.state           = TAKEOFF
        self.tag_pos         = None   # (p.x, p.y, p.z) in optical frame
        self.tag_seen        = False
        self.center_ok_count = 0
        self._lost_count     = 0
        self._hover_start    = None
        self.rate            = rospy.Rate(20)

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_z = 0.0

        # Running position setpoint integrated from velocity commands
        self._cmd_x = 0.0
        self._cmd_y = 0.0
        self._cmd_z = 0.0

        # Standalone demo: starts enabled.  Full mission: LANDER_INITIALLY_ENABLED=false.
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
        dt = 1.0 / 20
        self._cmd_x += vx * dt
        self._cmd_y += vy * dt
        self._cmd_z += vz * dt
        self.publish_pose(self._cmd_x, self._cmd_y, self._cmd_z)

    def hold(self):
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

            # ── HOVER ─────────────────────────────────────────────────────────
            elif self.state == HOVER:
                self.publish_pose(self.cur_x, self.cur_y, self.takeoff_alt)
                elapsed = (rospy.Time.now() - self._hover_start).to_sec()
                rospy.loginfo_throttle(1, f'[HOVER] stable {elapsed:.1f}/{self.hover_sec}s')
                if elapsed >= self.hover_sec:
                    self._cmd_x = self.cur_x
                    self._cmd_y = self.cur_y
                    self._cmd_z = self.cur_z
                    self.transition(SEARCHING)

            # ── SEARCHING ─────────────────────────────────────────────────────
            elif self.state == SEARCHING:
                self.hold()
                rospy.loginfo_throttle(2, '[SEARCHING] waiting for AprilTag (downward cam)...')
                if self.tag_seen:
                    self.transition(CENTERING)

            # ── CENTERING ─────────────────────────────────────────────────────
            elif self.state == CENTERING:
                if not self.tag_seen:
                    self._lost_count += 1
                    if self._lost_count > 60:
                        rospy.logwarn_throttle(3, '[Lander] Tag lost — holding')
                    self.hold()
                    self.rate.sleep()
                    continue
                self._lost_count = 0

                px, py = self.tag_pos[0], self.tag_pos[1]
                # Downward camera (rpy="0 pi/2 0"):
                #   optical x → body −Y:  vy = −kp × px
                #   optical y → body −X:  vx = −kp × py
                vy = self.clamp(-self.kp_xy * px, self.max_vxy)
                vx = self.clamp(-self.kp_xy * py, self.max_vxy)

                rospy.loginfo_throttle(1,
                    f'[CENTERING] px={px:.3f} py={py:.3f}  '
                    f'vx={vx:.2f} vy={vy:.2f}')
                self.update_cmd(vx=vx, vy=vy, vz=0.)

                if abs(px) < self.center_thr and abs(py) < self.center_thr:
                    self.center_ok_count += 1
                    if self.center_ok_count > 10:   # stable 0.5 s at 20 Hz
                        self.transition(DESCENDING)
                else:
                    self.center_ok_count = 0

            # ── DESCENDING ────────────────────────────────────────────────────
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

                px, py = self.tag_pos[0], self.tag_pos[1]
                vy = self.clamp(-self.kp_xy * px * 0.6, self.max_vxy * 0.5)
                vx = self.clamp(-self.kp_xy * py * 0.6, self.max_vxy * 0.5)

                if abs(px) > self.center_thr * 2.5 or abs(py) > self.center_thr * 2.5:
                    self.transition(CENTERING)
                    self.rate.sleep()
                    continue

                rospy.loginfo_throttle(0.5,
                    f'[DESCENDING] alt={self.cur_z:.2f}m  '
                    f'px={px:.3f} py={py:.3f}  '
                    f'cmd=({vx:.2f},{vy:.2f},{self.descent_vel:.2f})')
                self.update_cmd(vx=vx, vy=vy, vz=self.descent_vel)

                if self.cur_z < self.land_z:
                    rospy.loginfo(f'[Lander] Landed!  alt={self.cur_z:.2f}m')
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
