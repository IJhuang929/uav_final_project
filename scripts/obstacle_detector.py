#!/usr/bin/env python3
"""
obstacle_detector.py

Subscribes: /iris/vi_sensor/camera_depth/points  (sensor_msgs/PointCloud2)
Publishes:  /obstacle/info  (std_msgs/Float32MultiArray)
    data[0]  detected    1.0 = obstacle inside alert range, 0.0 = clear
    data[1]  distance    metres to nearest ROI point (1e9 if clear)
    data[2]  left_clear  min dist on port  side (x < 0 in optical frame; 1e9 = open)
    data[3]  right_clear min dist on starboard side (x ≥ 0;              1e9 = open)
    data[4]  gap_y       world-frame Y offset of best passable gap centre (0 = centre)
    data[5]  gap_width   passable width of that gap in metres (0 = none found)

Camera optical frame (vi_sensor faces forward in drone body):
    z = depth / forward    x = right    y = down
Port/starboard mapping: camera x < 0 = drone left = world +Y
                        camera x ≥ 0 = drone right = world -Y
"""
import math
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray

# ── ROI tuning ────────────────────────────────────────────────────────────────
DEPTH_MIN  = 0.40          # m  stereo near-field noise floor
DEPTH_MAX  = 6.00          # m  max reliable stereo range
HORIZ_CONE = math.radians(35)  # half-angle of horizontal detection cone
VERT_UP    = math.radians(20)  # max upward angle included (above optical axis)
VERT_DOWN  = math.radians(25)  # max downward angle included (towards ground)

ALERT_DIST = 3.00          # m  nearest ROI point < this  →  detected = 1
MIN_POINTS = 15            # fewer ROI points than this  →  treat as noise / clear
SUBSAMPLE  = 4             # stride: process every Nth point row (CPU budget)
INF        = 1e9           # sentinel "no obstacle" distance

# ── Gap detection tuning ──────────────────────────────────────────────────────
GAP_BINS       = 24        # horizontal bins across FOV for gap search
GAP_DEPTH_BAND = 0.8       # m — depth range around nearest obstacle to examine
GAP_MIN_PTS    = 2         # points per bin to count the bin as blocked
GAP_MIN_WIDTH  = 1.0       # m — minimum gap width to report as passable

CLOUD_TOPIC = '/iris/vi_sensor/camera_depth/points'
INFO_TOPIC  = '/obstacle/info'


class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=False)

        self.pub = rospy.Publisher(INFO_TOPIC, Float32MultiArray, queue_size=1)

        rospy.Subscriber(
            CLOUD_TOPIC, PointCloud2, self._cloud_cb,
            queue_size=1, buff_size=2 ** 24)

        rospy.loginfo(f'[ObsDet] Listening on {CLOUD_TOPIC}')
        rospy.loginfo(f'[ObsDet] ROI depth=[{DEPTH_MIN},{DEPTH_MAX}]m  '
                      f'horiz=±{math.degrees(HORIZ_CONE):.0f}°  '
                      f'alert<{ALERT_DIST}m')

    # ── Fast PointCloud2 → numpy ──────────────────────────────────────────────

    @staticmethod
    def _cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
        offsets = {f.name: f.offset for f in msg.fields}
        n    = msg.width * msg.height
        step = msg.point_step

        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        buf = buf[::SUBSAMPLE]

        def _f32(off: int) -> np.ndarray:
            return np.frombuffer(
                buf[:, off:off + 4].tobytes(), dtype=np.float32)

        return np.column_stack([
            _f32(offsets['x']),
            _f32(offsets['y']),
            _f32(offsets['z']),
        ])

    # ── Gap detection ─────────────────────────────────────────────────────────

    def _find_gap(self, x_roi: np.ndarray, z_roi: np.ndarray,
                  obs_dist: float) -> tuple[float, float]:
        """Find the widest horizontal gap through the obstacle field.

        Examines points within GAP_DEPTH_BAND of the nearest obstacle.
        Returns (gap_center_y, gap_width) in world-frame metres.
        gap_center_y: world Y offset of gap centre relative to drone
                      (positive = left / port, negative = right / starboard)
        gap_width:    passable width in metres; 0 = no gap found.
        """
        band = (z_roi >= obs_dist - GAP_DEPTH_BAND) & \
               (z_roi <= obs_dist + GAP_DEPTH_BAND)
        x_near = x_roi[band]

        if x_near.size == 0:
            return 0.0, 0.0

        # Bin edges spanning the horizontal FOV at the obstacle depth
        half_w    = obs_dist * math.tan(HORIZ_CONE)
        bin_edges = np.linspace(-half_w, half_w, GAP_BINS + 1)
        bin_w     = bin_edges[1] - bin_edges[0]

        counts, _ = np.histogram(x_near, bins=bin_edges)
        blocked   = counts >= GAP_MIN_PTS

        # Find the widest contiguous run of unblocked bins
        best_start, best_len = -1, 0
        cur_start,  cur_len  = -1, 0
        for i in range(GAP_BINS):
            if not blocked[i]:
                if cur_start < 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len  = cur_len
                    best_start = cur_start
            else:
                cur_start, cur_len = -1, 0

        if best_start < 0:
            return 0.0, 0.0

        gap_center_x = (bin_edges[best_start] +
                        bin_edges[best_start + best_len]) / 2.0
        gap_width_x  = best_len * bin_w

        # Camera: x < 0 = left = world +Y → negate
        return float(-gap_center_x), float(gap_width_x)

    # ── Callback ──────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2):
        clear_msg = Float32MultiArray(data=[0.0, INF, INF, INF, 0.0, 0.0])

        field_names = {f.name for f in msg.fields}
        if not {'x', 'y', 'z'}.issubset(field_names):
            rospy.logwarn_throttle(10.0, '[ObsDet] PointCloud2 missing x/y/z fields')
            self.pub.publish(clear_msg)
            return

        try:
            pts = self._cloud_to_xyz(msg)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, f'[ObsDet] Cloud parse error: {exc}')
            self.pub.publish(clear_msg)
            return

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

        # 1. Remove NaN / inf
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]

        if z.size == 0:
            self.pub.publish(clear_msg)
            return

        # 2. Depth band
        mask = (z > DEPTH_MIN) & (z < DEPTH_MAX)

        # 3. Horizontal cone
        mask &= np.arctan2(np.abs(x), z) < HORIZ_CONE

        # 4. Vertical band
        vert  = np.arctan2(y, z)
        mask &= (vert > -VERT_UP) & (vert < VERT_DOWN)

        x_roi = x[mask]
        z_roi = z[mask]

        if x_roi.size < MIN_POINTS:
            self.pub.publish(clear_msg)
            rospy.logdebug(f'[ObsDet] roi_pts={x_roi.size} < {MIN_POINTS} — clear')
            return

        # 5. Distances
        min_dist = float(z_roi.min())

        left_mask  = x_roi <  0.0
        right_mask = x_roi >= 0.0
        left_clear  = float(z_roi[left_mask].min())  if left_mask.any()  else INF
        right_clear = float(z_roi[right_mask].min()) if right_mask.any() else INF

        detected = 1.0 if min_dist < ALERT_DIST else 0.0

        # 6. Gap detection (only when obstacle is in alert zone)
        if detected:
            gap_y, gap_w = self._find_gap(x_roi, z_roi, min_dist)
        else:
            gap_y, gap_w = 0.0, 0.0

        out = Float32MultiArray(
            data=[detected, min_dist, left_clear, right_clear, gap_y, gap_w])
        self.pub.publish(out)

        rospy.loginfo_throttle(0.5,
            f'[ObsDet] det={detected:.0f}  dist={min_dist:.2f}m  '
            f'L={left_clear:.2f}m  R={right_clear:.2f}m  '
            f'gap_y={gap_y:+.2f}m  gap_w={gap_w:.2f}m')

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        ObstacleDetector().run()
    except rospy.ROSInterruptException:
        pass
