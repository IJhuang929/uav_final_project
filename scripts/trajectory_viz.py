#!/usr/bin/env python3
"""
trajectory_viz.py — Real-time trajectory visualisation for obstacle-course mission.

RViz MarkerArray on /viz/markers:
  ns='trail'     id=0  LINE_STRIP  — accumulated drone flight path
  ns='obstacles' id=1-5 CYLINDER   — P1-P5 pillar obstacles
  ns='apriltag'  id=10  ARROW      — landing pad indicator (pointing down)
                 id=11  TEXT       — label

Gazebo: spawns spheres via /gazebo/spawn_sdf_model
  Blue (r=0.04): drone actual flight trail       (every TRAIL_SPACING=0.5 m, historical)
  Red  (r=0.06): minimum-snap planned trajectory (every PLAN_SPACING=0.3 m,
                 refreshed from /traj/planned_path on each replan)
"""
import math
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion

# ── World constants ────────────────────────────────────────────────────────────
GRID_RES     = 0.1    # m per cell
GRID_OX      = -1.0   # world X of grid left edge
GRID_OY      = -6.0   # world Y of grid bottom edge
GRID_W       = 180    # cells in X  (18 m)
GRID_H       = 120    # cells in Y  (12 m)
FREE_RADIUS  = 0.8    # m — radius around drone path to mark as free

OBSTACLES = [
    # (x,    y,    radius, height)
    ( 4.0,   0.0,  0.40,  3.0),
    ( 6.5,   1.8,  0.35,  3.0),
    ( 7.0,  -1.5,  0.35,  3.0),
    ( 9.5,   0.6,  0.40,  3.0),
    (11.5,  -1.0,  0.30,  3.0),
]
APRILTAG_XY  = (14.0, 0.0)
TRAIL_SPACING   = 0.5   # m between blue Gazebo trail spheres
PLAN_SPACING    = 0.3   # m between red planned-path spheres (sub-sampled from dense traj)
MAX_PLAN_AHEAD  = 12    # max red spheres to show (≈ PLAN_SPACING × MAX_PLAN_AHEAD m look-ahead)


class TrajectoryViz:
    def __init__(self):
        rospy.init_node('trajectory_viz', anonymous=False)

        self.marker_pub = rospy.Publisher(
            '/viz/markers', MarkerArray, queue_size=1, latch=True)
        self.occ_pub = rospy.Publisher(
            '/viz/occupancy_map', OccupancyGrid, queue_size=1, latch=True)

        self._occ = [-1] * (GRID_W * GRID_H)
        self._init_occ_obstacles()

        rospy.loginfo('[VizNode] Waiting for Gazebo services...')
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30.0)
        rospy.wait_for_service('/gazebo/delete_model',    timeout=10.0)
        self.spawn_srv  = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model',    DeleteModel)

        # ── Static markers (obstacles + AprilTag) — merged into every trail publish
        self._static_markers: list[Marker] = []

        # ── Trail state ───────────────────────────────────────────────────────
        self.trail: list[Point] = []
        self.last_sphere_pos  = None
        self.sphere_count     = 0

        # ── Planned-path state ────────────────────────────────────────────────
        self.planned_names: list[str] = []   # Gazebo model names to delete on replan
        self.planned_count = 0

        rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_cb)
        rospy.Subscriber('/traj/planned_path',   Path,        self._planned_path_cb)

        # Publish static markers (obstacles + AprilTag) after Gazebo settles
        rospy.Timer(rospy.Duration(3.0), self._publish_static, oneshot=True)
        rospy.Timer(rospy.Duration(0.5), self._publish_occ)

        rospy.loginfo('[VizNode] Ready')

    # ── Drone trail ───────────────────────────────────────────────────────────

    def _model_states_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index('iris')
        except ValueError:
            return

        p = msg.pose[idx].position
        pt = Point(p.x, p.y, p.z)
        self.trail.append(pt)
        self._mark_free_around(p.x, p.y)

        if (self.last_sphere_pos is None or
                self._dist3(pt, self.last_sphere_pos) >= TRAIL_SPACING):
            self._spawn_sphere(pt)
            self.last_sphere_pos = pt

        self._publish_trail()

    # ── Planned path (red dense dots from minimum-snap trajectory) ───────────

    def _planned_path_cb(self, msg: Path):
        # Delete previous planned-path spheres
        for name in self.planned_names:
            try:
                self.delete_srv(name)
            except Exception:
                pass
        self.planned_names.clear()

        # Sub-sample: one sphere every PLAN_SPACING m, up to MAX_PLAN_AHEAD spheres
        last_pt: Point | None = None
        for ps in msg.poses:
            if len(self.planned_names) >= MAX_PLAN_AHEAD:
                break
            p  = ps.pose.position
            pt = Point(p.x, p.y, p.z)
            if last_pt is None or self._dist3(pt, last_pt) >= PLAN_SPACING:
                name = f'plan_{self.planned_count}'
                self.planned_count += 1
                self._spawn_planned_sphere(pt, name)
                self.planned_names.append(name)
                last_pt = pt

    # ── Occupancy grid ────────────────────────────────────────────────────────

    def _cell(self, wx, wy):
        return int((wx - GRID_OX) / GRID_RES), int((wy - GRID_OY) / GRID_RES)

    def _init_occ_obstacles(self):
        for (ox, oy, radius, _) in OBSTACLES:
            cx0, cy0 = self._cell(ox, oy)
            r_c = int(math.ceil(radius / GRID_RES)) + 1
            for dx in range(-r_c, r_c + 1):
                for dy in range(-r_c, r_c + 1):
                    if (dx * dx + dy * dy) ** 0.5 * GRID_RES <= radius:
                        ix, iy = cx0 + dx, cy0 + dy
                        if 0 <= ix < GRID_W and 0 <= iy < GRID_H:
                            self._occ[iy * GRID_W + ix] = 100

    def _mark_free_around(self, wx, wy):
        cx, cy = self._cell(wx, wy)
        r_c = int(FREE_RADIUS / GRID_RES)
        for dx in range(-r_c, r_c + 1):
            for dy in range(-r_c, r_c + 1):
                if (dx * dx + dy * dy) ** 0.5 * GRID_RES <= FREE_RADIUS:
                    ix, iy = cx + dx, cy + dy
                    if 0 <= ix < GRID_W and 0 <= iy < GRID_H:
                        if self._occ[iy * GRID_W + ix] != 100:
                            self._occ[iy * GRID_W + ix] = 0

    def _publish_occ(self, _event=None):
        msg = OccupancyGrid()
        msg.header.frame_id = 'world'
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution = GRID_RES
        msg.info.width  = GRID_W
        msg.info.height = GRID_H
        msg.info.origin.position.x = GRID_OX
        msg.info.origin.position.y = GRID_OY
        msg.info.origin.orientation.w = 1.0
        msg.data = self._occ
        self.occ_pub.publish(msg)

    # ── RViz publishers ───────────────────────────────────────────────────────

    def _publish_trail(self):
        if len(self.trail) < 2:
            return
        trail_m = Marker()
        trail_m.header.frame_id = 'world'
        trail_m.header.stamp = rospy.Time.now()
        trail_m.ns = 'trail'
        trail_m.id = 0
        trail_m.type = Marker.LINE_STRIP
        trail_m.action = Marker.ADD
        trail_m.scale.x = 0.04
        trail_m.color.r = 0.2
        trail_m.color.g = 0.8
        trail_m.color.b = 1.0
        trail_m.color.a = 0.9
        trail_m.points = list(self.trail)
        trail_m.lifetime = rospy.Duration(0)

        # Always include static markers so the latched message is complete
        ma = MarkerArray()
        ma.markers = self._static_markers + [trail_m]
        self.marker_pub.publish(ma)

    def _publish_static(self, _event=None):
        ma = MarkerArray()

        # Obstacle cylinders
        for i, (ox, oy, radius, height) in enumerate(OBSTACLES):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = rospy.Time.now()
            m.ns = 'obstacles'
            m.id = i + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = height / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = radius * 2
            m.scale.y = radius * 2
            m.scale.z = height
            m.color.r = 1.0
            m.color.g = 0.4
            m.color.b = 0.0
            m.color.a = 0.5
            m.lifetime = rospy.Duration(0)
            ma.markers.append(m)

        # AprilTag — downward arrow
        arrow = Marker()
        arrow.header.frame_id = 'world'
        arrow.header.stamp = rospy.Time.now()
        arrow.ns = 'apriltag'
        arrow.id = 10
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        ax, ay = APRILTAG_XY
        arrow.points = [
            Point(ax, ay, 2.2),
            Point(ax, ay, 0.05),
        ]
        arrow.scale.x = 0.08
        arrow.scale.y = 0.20
        arrow.scale.z = 0.30
        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 0.3
        arrow.color.a = 1.0
        arrow.lifetime = rospy.Duration(0)
        ma.markers.append(arrow)

        # AprilTag — text label
        label = Marker()
        label.header.frame_id = 'world'
        label.header.stamp = rospy.Time.now()
        label.ns = 'apriltag'
        label.id = 11
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = ax
        label.pose.position.y = ay
        label.pose.position.z = 2.7
        label.pose.orientation.w = 1.0
        label.scale.z = 0.35
        label.color.r = 0.0
        label.color.g = 1.0
        label.color.b = 0.3
        label.color.a = 1.0
        label.text = 'AprilTag\ntag36h11 id=0'
        label.lifetime = rospy.Duration(0)
        ma.markers.append(label)

        self._static_markers = list(ma.markers)   # cache for inclusion in trail publishes
        self.marker_pub.publish(ma)
        rospy.loginfo('[VizNode] Static markers published (5 obstacles + AprilTag)')

    # ── Gazebo sphere helpers ──────────────────────────────────────────────────

    def _sphere_sdf(self, name: str, r: float, rgba: tuple) -> str:
        ri, gi, bi, ai = rgba
        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry><sphere><radius>{r}</radius></sphere></geometry>
        <material>
          <ambient>{ri} {gi} {bi} {ai}</ambient>
          <diffuse>{ri} {gi} {bi} {ai}</diffuse>
          <specular>0 0 0 0</specular>
          <emissive>0 0 0 0</emissive>
        </material>
        <transparency>{1.0 - ai}</transparency>
      </visual>
    </link>
  </model>
</sdf>"""

    def _spawn_sphere(self, pos: Point):
        name = f'trail_{self.sphere_count}'
        self.sphere_count += 1
        sdf = self._sphere_sdf(name, r=0.04, rgba=(0.2, 0.7, 1.0, 0.5))
        self._do_spawn(name, sdf, pos)

    def _spawn_planned_sphere(self, pos: Point, name: str):
        sdf = self._sphere_sdf(name, r=0.04, rgba=(1.0, 0.15, 0.15, 0.85))
        self._do_spawn(name, sdf, pos)

    def _do_spawn(self, name: str, sdf: str, pos: Point):
        model_pose = Pose()
        model_pose.position.x = pos.x
        model_pose.position.y = pos.y
        model_pose.position.z = pos.z
        model_pose.orientation = Quaternion(0, 0, 0, 1)
        try:
            self.spawn_srv(name, sdf, '', model_pose, 'world')
        except Exception as e:
            rospy.logwarn_throttle(5.0, f'[VizNode] Spawn failed: {e}')

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _dist3(a: Point, b: Point) -> float:
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        TrajectoryViz().run()
    except rospy.ROSInterruptException:
        pass
