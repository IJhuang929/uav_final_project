#!/usr/bin/env python3
"""
trajectory_planner.py — Minimum Snap trajectory planner ROS node.

Subscriptions:
  /traj/waypoints  nav_msgs/Path    — 3-D poses to fly through; triggers replan.
                                      header.stamp.to_sec() encodes total_time (s);
                                      0 → use ~total_time ROS param (default 20 s).
  /traj/cancel     std_msgs/Empty   — abort current trajectory → IDLE.

Publications:
  /iris/command/pose  geometry_msgs/PoseStamped  — setpoints at ~publish_hz Hz
  /traj/status        std_msgs/String            — "IDLE" | "EXECUTING" | "DONE" | "FAILED"

ROS params:
  ~total_time   float  default 20.0
  ~publish_hz   float  default 20.0
  ~mav_name     string default "iris"

Standalone test (no ROS):
  python3 trajectory_planner.py
"""
from __future__ import annotations
import threading
import numpy as np

try:
    from cvxopt import matrix, solvers
    solvers.options['show_progress'] = False
except ImportError:
    raise ImportError("cvxopt missing — pip install cvxopt --break-system-packages")

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import Empty, String
    _ROS = True
except ImportError:
    _ROS = False

N_ORDER   = 7    # polynomial order; 7 is required for snap (4th-deriv) minimisation
VEL_SCALE = 1.5  # junction velocity bound = avg_speed_of_segment × VEL_SCALE


# ── Math core (unchanged from original) ───────────────────────────────────────

def _dpoly_row(order, t, deriv):
    """Row vector for the deriv-th derivative of an order-degree polynomial at t."""
    row = np.zeros(order + 1)
    for col in range(order + 1):
        power = order - col
        if power >= deriv:
            coeff = 1
            for d in range(deriv):
                coeff *= (power - d)
            row[col] = coeff * (t ** (power - deriv))
    return row


def get_Q(n_seg, n_order, ts):
    """Block-diagonal snap-cost matrix (analytic integral of squared 4th derivative)."""
    blocks = []
    for k in range(n_seg):
        T = ts[k]
        Q_k = np.array([
            [100800*T**7, 50400*T**6, 20160*T**5,  5040*T**4, 0, 0, 0, 0],
            [ 50400*T**6, 25920*T**5, 10800*T**4,  2880*T**3, 0, 0, 0, 0],
            [ 20160*T**5, 10800*T**4,  4800*T**3,  1440*T**2, 0, 0, 0, 0],
            [  5040*T**4,  2880*T**3,  1440*T**2,   576*T**1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
        ], dtype=float)
        blocks.append(Q_k)
    n = n_seg * (n_order + 1)
    Q = np.zeros((n, n))
    for i, blk in enumerate(blocks):
        s = i * (n_order + 1)
        Q[s:s+n_order+1, s:s+n_order+1] = blk
    return Q


def get_Aeq_beq(n_seg, n_order, waypoints, ts, start_cond, end_cond):
    """Equality constraint matrix: start/end derivatives + waypoints + continuity."""
    n_coef = n_seg * (n_order + 1)

    Aeq_start = np.zeros((4, n_coef))
    for deriv in range(4):
        Aeq_start[deriv, :n_order+1] = _dpoly_row(n_order, 0.0, deriv)
    beq_start = np.array(start_cond, dtype=float)

    Aeq_end = np.zeros((4, n_coef))
    for deriv in range(4):
        Aeq_end[deriv, -n_order-1:] = _dpoly_row(n_order, ts[-1], deriv)
    beq_end = np.array(end_cond, dtype=float)

    Aeq_wp  = np.zeros((n_seg - 1, n_coef))
    beq_wp  = np.zeros(n_seg - 1)
    for m in range(n_seg - 1):
        idx = m * (n_order + 1)
        Aeq_wp[m, idx:idx+n_order+1] = _dpoly_row(n_order, ts[m], 0)
        beq_wp[m] = waypoints[m + 1]

    con_rows, con_rhs = [], []
    for deriv in range(4):
        A = np.zeros((n_seg - 1, n_coef))
        for m in range(n_seg - 1):
            idx = m * (n_order + 1)
            A[m, idx:idx+n_order+1]             =  _dpoly_row(n_order, ts[m],  deriv)
            A[m, idx+n_order+1:idx+2*(n_order+1)] = -_dpoly_row(n_order, 0.0, deriv)
        con_rows.append(A)
        con_rhs.append(np.zeros(n_seg - 1))

    Aeq = np.vstack([Aeq_start, Aeq_end, Aeq_wp] + con_rows)
    beq = np.concatenate([beq_start, beq_end, beq_wp] + con_rhs)
    return Aeq, beq


def get_Gineq_hineq(n_seg, n_order, ts, v_max_junctions):
    """Inequality constraints: -v_max[m] ≤ vel at junction m ≤ +v_max[m].

    Each internal junction m (end of segment m) contributes two rows:
      +dpoly_row(ts[m], 1) · c_m  ≤ +v_max[m]
      -dpoly_row(ts[m], 1) · c_m  ≤ +v_max[m]

    Returns G (2*(n_seg-1), n_coef) and h (2*(n_seg-1),).
    """
    n_junctions = n_seg - 1
    n_coef      = n_seg * (n_order + 1)
    G = np.zeros((2 * n_junctions, n_coef))
    h = np.zeros(2 * n_junctions)
    for m in range(n_junctions):
        r = np.zeros(n_coef)
        r[m*(n_order+1):(m+1)*(n_order+1)] = _dpoly_row(n_order, ts[m], deriv=1)
        G[2*m,   :] = +r
        G[2*m+1, :] = -r
        h[2*m]      = v_max_junctions[m]
        h[2*m+1]    = v_max_junctions[m]
    return G, h


def compute_v_max(waypoints_3d, ts):
    """Per-junction velocity bound (shape n_seg-1).

    v_max[m] = (3-D distance of segment m / ts[m]) * VEL_SCALE

    Using the segment that ENDS at each junction keeps the bound tight:
    the polynomial must arrive at the waypoint at roughly the average speed
    of the preceding segment, scaled up by VEL_SCALE to give the QP room
    to minimise snap while preventing physically unreasonable overshoots.
    """
    dists = np.linalg.norm(np.diff(np.asarray(waypoints_3d), axis=0), axis=1)
    return (dists[:-1] / ts[:-1]) * VEL_SCALE


def minimum_snap_qp(waypoints_1d, ts, n_seg, n_order, v_max_junctions):
    """Solve minimum-snap QP for one axis. Returns coefficient vector."""
    start_cond = [waypoints_1d[0],  0.0, 0.0, 0.0]
    end_cond   = [waypoints_1d[-1], 0.0, 0.0, 0.0]

    Q        = get_Q(n_seg, n_order, ts)
    Aeq, beq = get_Aeq_beq(n_seg, n_order, waypoints_1d, ts, start_cond, end_cond)

    n     = Q.shape[0]
    Q_reg = Q + 1e-8 * np.eye(n)

    if n_seg > 1:
        G, h = get_Gineq_hineq(n_seg, n_order, ts, v_max_junctions)
        sol  = solvers.qp(matrix(Q_reg), matrix(np.zeros(n)),
                          G=matrix(G), h=matrix(h),
                          A=matrix(Aeq), b=matrix(beq))
    else:
        sol = solvers.qp(matrix(Q_reg), matrix(np.zeros(n)),
                         A=matrix(Aeq), b=matrix(beq))

    if sol['status'] != 'optimal':
        raise RuntimeError(f"QP status: {sol['status']}")
    return np.array(sol['x']).flatten()


def eval_trajectory(coef_x, coef_y, coef_z, ts, n_seg, n_order, dt=0.05):
    """Sample trajectory at dt resolution. Returns list[(x,y,z)] and list[t_abs]."""
    points, times = [], []
    t_abs = 0.0
    for i in range(n_seg):
        s  = i * (n_order + 1)
        px = coef_x[s:s+n_order+1]
        py = coef_y[s:s+n_order+1]
        pz = coef_z[s:s+n_order+1]
        t  = 0.0
        while t <= ts[i] + 1e-9:
            points.append((float(np.polyval(px, t)),
                           float(np.polyval(py, t)),
                           float(np.polyval(pz, t))))
            times.append(t_abs + t)
            t += dt
        t_abs += ts[i]
    return points, times


def allocate_time(waypoints, total_time):
    """Distribute total_time across segments proportionally to Euclidean distance."""
    pts   = np.array(waypoints)
    dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    ts    = dists / dists.sum() * total_time
    return np.maximum(ts, 0.1)   # guard against zero-length segments


# ── ROS node ──────────────────────────────────────────────────────────────────

class TrajectoryPlanner:

    _STATUS_IDLE      = 'IDLE'
    _STATUS_EXECUTING = 'EXECUTING'
    _STATUS_DONE      = 'DONE'
    _STATUS_FAILED    = 'FAILED'

    def __init__(self):
        rospy.init_node('trajectory_planner', anonymous=False)

        ns            = rospy.get_param('~mav_name',   'iris')
        self._default_total_time = rospy.get_param('~total_time', 20.0)
        self._publish_hz         = rospy.get_param('~publish_hz', 20.0)

        self._pose_pub   = rospy.Publisher(
            f'/{ns}/command/pose', PoseStamped, queue_size=1)
        self._status_pub = rospy.Publisher(
            '/traj/status', String, queue_size=1, latch=True)
        self._planned_path_pub = rospy.Publisher(
            '/traj/planned_path', Path, queue_size=1, latch=True)

        rospy.Subscriber('/traj/waypoints', Path,  self._waypoints_cb)
        rospy.Subscriber('/traj/cancel',    Empty, self._cancel_cb)

        self._lock       = threading.Lock()
        self._stop_event = threading.Event()
        self._exec_thread: threading.Thread | None = None

        self._publish_status(self._STATUS_IDLE)
        rospy.loginfo('[TrajPlanner] Ready — waiting for /traj/waypoints')

    # ── status ────────────────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        self._status_pub.publish(String(data=status))
        rospy.loginfo(f'[TrajPlanner] Status → {status}')

    # ── cancel ────────────────────────────────────────────────────────────────

    def _cancel_cb(self, _msg):
        self._stop_execution()
        self._publish_status(self._STATUS_IDLE)

    def _stop_execution(self):
        self._stop_event.set()
        with self._lock:
            if self._exec_thread and self._exec_thread.is_alive():
                self._exec_thread.join(timeout=1.0)
            self._exec_thread = None
        self._stop_event.clear()

    # ── new waypoints → replan ────────────────────────────────────────────────

    def _waypoints_cb(self, msg: Path):
        if len(msg.poses) < 2:
            rospy.logwarn('[TrajPlanner] Need at least 2 waypoints — ignored')
            return

        # Decode total_time from header.stamp (0 → use default param)
        total_time = msg.header.stamp.to_sec()
        if total_time <= 0.0:
            total_time = self._default_total_time

        waypoints = np.array([[p.pose.position.x,
                                p.pose.position.y,
                                p.pose.position.z] for p in msg.poses])

        rospy.loginfo(f'[TrajPlanner] Replanning: {len(waypoints)} waypoints, '
                      f'total_time={total_time:.1f}s')

        # Solve QP (blocking, ~50 ms) before touching the execution thread
        try:
            points, times = self._solve(waypoints, total_time)
        except Exception as exc:
            rospy.logerr(f'[TrajPlanner] QP failed: {exc}')
            self._publish_status(self._STATUS_FAILED)
            return

        # Publish full dense path for visualisation (trajectory_viz red dots)
        self._publish_planned_path(points)

        # Swap out current trajectory atomically
        self._stop_execution()

        self._stop_event.clear()
        t = threading.Thread(target=self._execute_loop,
                             args=(points, times), daemon=True)
        with self._lock:
            self._exec_thread = t
        t.start()

    # ── Planned path publisher (for trajectory_viz red dots) ─────────────────

    def _publish_planned_path(self, points: list):
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp    = rospy.Time.now()
        for x, y, z in points:
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._planned_path_pub.publish(path)

    # ── QP solve (called from waypoints_cb, not in a thread) ─────────────────

    def _solve(self, waypoints, total_time):
        n_seg = len(waypoints) - 1
        ts    = allocate_time(waypoints, total_time)
        v_max = compute_v_max(waypoints, ts)

        rospy.loginfo(f'[TrajPlanner] Solving QP ({n_seg} segments)  '
                      f'ts={np.round(ts, 2)}  v_max={np.round(v_max, 2)} m/s')

        coef_x = minimum_snap_qp(waypoints[:, 0], ts, n_seg, N_ORDER, v_max)
        coef_y = minimum_snap_qp(waypoints[:, 1], ts, n_seg, N_ORDER, v_max)
        coef_z = minimum_snap_qp(waypoints[:, 2], ts, n_seg, N_ORDER, v_max)

        dt = 1.0 / self._publish_hz
        points, times = eval_trajectory(coef_x, coef_y, coef_z,
                                        ts, n_seg, N_ORDER, dt=dt)
        rospy.loginfo(f'[TrajPlanner] {len(points)} setpoints ready')
        return points, times

    # ── execution loop (runs in background thread) ────────────────────────────

    def _execute_loop(self, points, times):
        self._publish_status(self._STATUS_EXECUTING)
        rate   = rospy.Rate(self._publish_hz)
        t0     = rospy.Time.now()

        for (x, y, z), t_rel in zip(points, times):
            if self._stop_event.is_set() or rospy.is_shutdown():
                return   # cancelled — caller will set status

            msg = PoseStamped()
            msg.header.stamp    = t0 + rospy.Duration(t_rel)
            msg.header.frame_id = 'world'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.w = 1.0

            self._pose_pub.publish(msg)
            rate.sleep()

        self._publish_status(self._STATUS_DONE)

    def run(self):
        rospy.spin()


# ── standalone test (no ROS) ──────────────────────────────────────────────────

def standalone_test():
    waypoints = np.array([
        [0.0,  0.0,  1.5],
        [4.0,  0.0,  1.5],
        [6.5,  1.8,  1.5],
        [9.5,  0.6,  1.5],
        [11.5, -1.0, 1.5],
        [14.0,  0.0, 1.5],
    ])
    total_time = 20.0
    n_seg = len(waypoints) - 1
    ts    = allocate_time(waypoints, total_time)
    v_max = compute_v_max(waypoints, ts)

    print(f'Segments   : {n_seg}')
    print(f'Time alloc : {np.round(ts, 3)}')
    print(f'v_max      : {np.round(v_max, 3)} m/s')

    coef_x = minimum_snap_qp(waypoints[:, 0], ts, n_seg, N_ORDER, v_max)
    coef_y = minimum_snap_qp(waypoints[:, 1], ts, n_seg, N_ORDER, v_max)
    coef_z = minimum_snap_qp(waypoints[:, 2], ts, n_seg, N_ORDER, v_max)

    points, times = eval_trajectory(coef_x, coef_y, coef_z, ts, n_seg, N_ORDER, dt=0.05)

    print(f'Setpoints  : {len(points)}')
    print(f'First      : {tuple(round(v,3) for v in points[0])}')
    print(f'Last       : {tuple(round(v,3) for v in points[-1])}')
    print(f'Duration   : {times[-1]:.2f} s')

    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        xs, ys, zs = zip(*points)
        fig = plt.figure()
        ax  = fig.add_subplot(111, projection='3d')
        ax.plot(xs, ys, zs, 'b-', linewidth=1.5, label='trajectory')
        ax.scatter(waypoints[:,0], waypoints[:,1], waypoints[:,2],
                   c='r', s=60, zorder=5, label='waypoints')
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_title('Minimum Snap Trajectory')
        ax.legend()
        plt.tight_layout()
        plt.show()
    except Exception:
        pass


if __name__ == '__main__':
    if not _ROS:
        print('[TrajPlanner] No ROS — running standalone test')
        standalone_test()
    else:
        try:
            TrajectoryPlanner().run()
        except rospy.ROSInitException:
            print('[TrajPlanner] No ROS master — running standalone test')
            standalone_test()
