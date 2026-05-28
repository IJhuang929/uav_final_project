#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# ── Source selector ───────────────────────────────────────────────────────────
# True  → subscribe to /rtabmap/odom (visual odometry)
# False → subscribe to /gazebo/model_states (ground truth, fallback)
# Override via env var:  USE_RTABMAP=false python3 gazebo_controller.py
import os as _os
USE_RTABMAP = _os.environ.get('USE_RTABMAP', 'true').lower() not in ('false', '0', 'no')

rospy.init_node('gazebo_pose_relay')

# Publishers — all sources write to the same three topics
pose_pub  = rospy.Publisher('/iris/ground_truth/pose',     PoseStamped, queue_size=1)
odom_pub  = rospy.Publisher('/iris/ground_truth/odometry', Odometry,    queue_size=1)
relay_pub = rospy.Publisher('/iris/odometry_sensor1/pose', PoseStamped, queue_size=1)


def _publish(pose, twist=None):
    now = rospy.Time.now()

    ps = PoseStamped()
    ps.header.stamp    = now
    ps.header.frame_id = 'world'
    ps.pose = pose
    pose_pub.publish(ps)
    relay_pub.publish(ps)

    odom = Odometry()
    odom.header.stamp    = now
    odom.header.frame_id = 'world'
    odom.child_frame_id  = 'iris/base_link'
    odom.pose.pose       = pose
    if twist is not None:
        odom.twist.twist = twist
    odom_pub.publish(odom)


if USE_RTABMAP:
    def _rtabmap_cb(msg: Odometry):
        _publish(msg.pose.pose, msg.twist.twist)

    rospy.Subscriber('/rtabmap/odom', Odometry, _rtabmap_cb)
    rospy.loginfo('[PoseRelay] source=/rtabmap/odom → /iris/ground_truth/* + /iris/odometry_sensor1/pose')
else:
    def _gazebo_cb(msg: ModelStates):
        try:
            idx = msg.name.index('iris')
            _publish(msg.pose[idx], msg.twist[idx])
        except ValueError:
            pass

    rospy.Subscriber('/gazebo/model_states', ModelStates, _gazebo_cb)
    rospy.loginfo('[PoseRelay] source=/gazebo/model_states (ground truth) → /iris/ground_truth/* + /iris/odometry_sensor1/pose')

rospy.spin()
