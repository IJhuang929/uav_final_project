#!/bin/bash
# demo_obstacle_world.sh
# ─────────────────────────────────────────────────────────────────────────────
# Launches obstacle_course.world with iris + vi_sensor in Gazebo.
# Use this to visually verify the world layout before running the full mission.
#
# World layout (bird's-eye):
#   Y
#   2  │          [P2]
#   1  │                    [P4]
#   0  │  [START] [P1]                   [AprilTag]
#  -1  │              [P3]  [P5]
#      └──────────────────────────────────  X
#          0   4  6.5  7  9.5 11.5       14

set -e
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash 2>/dev/null || true

export DISPLAY=:99

# ── 同步 world 檔到 roslaunch 實際讀取的路徑 ──────────────────────────────────
# roslaunch 用 $(find rotors_gazebo)/worlds/，不是 src/worlds/
# src/worlds 是 read-only mount，所以把最新版複製過去
cp /root/catkin_ws/src/worlds/obstacle_course.world \
   /root/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds/obstacle_course.world

# ── 確保 AprilTag 材質在 Gazebo 系統路徑（gzserver 100% 找得到）────────────────
cp /root/catkin_ws/src/worlds/media/materials/scripts/apriltag.material \
   /usr/share/gazebo-11/media/materials/scripts/ 2>/dev/null || true
cp /root/catkin_ws/src/worlds/media/materials/textures/tag36_11_00000.png \
   /usr/share/gazebo-11/media/materials/textures/ 2>/dev/null || true
export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:${GAZEBO_MODEL_PATH}
# worlds/ 也要在 RESOURCE_PATH 裡，Gazebo 才能找到 media/materials/scripts/apriltag.material
export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/worlds:/root/catkin_ws/src/rotors_simulator/rotors_gazebo:${GAZEBO_RESOURCE_PATH}

echo "======================================"
echo " Obstacle Course World"
echo " iris + vi_sensor"
echo "======================================"
echo ""
echo " Pillar layout (x, y):"
echo "   P1: (4.0,  0.0) r=0.40  center blocker"
echo "   P2: (6.5,  1.8) r=0.35  upper guard"
echo "   P3: (7.0, -1.5) r=0.35  lower guard"
echo "   P4: (9.5,  0.6) r=0.40  second center"
echo "   P5: (11.5,-1.0) r=0.30  final before tag"
echo "   AprilTag: (14.0, 0.0)"
echo ""

# ── Gazebo with obstacle_course world ─────────────────────────────────────────
xterm -title "Gazebo: Obstacle Course" \
  -geometry 110x30+10+50 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:\$GAZEBO_MODEL_PATH
    export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/worlds:/root/catkin_ws/src/rotors_simulator/rotors_gazebo:\$GAZEBO_RESOURCE_PATH

    roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch \
      mav_name:=iris \
      world_name:=obstacle_course 2>&1
    bash
  " &

# Wait for Gazebo + all models to load (obstacle_course is heavier than basic.world)
sleep 20

# ── Trajectory visualisation node ─────────────────────────────────────────────
xterm -title "Traj Viz Node" \
  -geometry 80x12+10+450 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    python3 /root/scripts/trajectory_viz.py 2>&1
    bash
  " &

sleep 2

# ── Trajectory planner node ───────────────────────────────────────────────────
xterm -title "Traj Planner" \
  -geometry 80x12+700+270 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    python3 /root/scripts/trajectory_planner.py 2>&1
    bash
  " &

sleep 2

# ── Obstacle detector node ────────────────────────────────────────────────────
xterm -title "Obstacle Detector" \
  -geometry 80x12+700+450 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    python3 /root/scripts/obstacle_detector.py 2>&1
    bash
  " &

sleep 2

# ── RViz with trajectory config ───────────────────────────────────────────────
xterm -title "RViz: Trajectory" \
  -geometry 80x12+10+620 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    rviz -d /root/scripts/trajectory_viz.rviz 2>&1
    bash
  " &

sleep 2

# ── Takeoff + test waypoint ────────────────────────────────────────────────────
# The launch file's hovering_example node has a 10-second timeout for Gazebo
# unpause. With a heavy world it often exits before the service is ready,
# leaving Gazebo paused and the drone on the ground. This block ensures the
# simulation is unpaused and sends an explicit takeoff + test-move command.
xterm -title "Takeoff & Waypoints" \
  -geometry 80x16+700+50 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99

    echo '--- Waiting for Lee controller topic ---'
    until rostopic list 2>/dev/null | grep -q '/iris/command/pose'; do sleep 1; done

    echo '--- Unpausing Gazebo (idempotent) ---'
    rosservice call /gazebo/unpause_physics

    sleep 2

    echo '--- Takeoff: z=1.5 ---'
    rostopic pub -1 /iris/command/pose geometry_msgs/PoseStamped \
      '{header: {frame_id: world}, pose: {position: {x: 0, y: 0, z: 1.5}, orientation: {w: 1}}}'

    sleep 5

    echo '--- Move to x=4 (trail test) ---'
    rostopic pub -1 /iris/command/pose geometry_msgs/PoseStamped \
      '{header: {frame_id: world}, pose: {position: {x: 4, y: 0, z: 1.5}, orientation: {w: 1}}}'

    echo ''
    echo 'Drone should now be flying. Send more waypoints here:'
    echo '  rostopic pub -1 /iris/command/pose geometry_msgs/PoseStamped \'\''{header:{frame_id:world},pose:{position:{x:?,y:?,z:1.5},orientation:{w:1}}}'\'\'
    bash
  " &

echo ""
echo "✅  Obstacle course + viz launching..."
echo "   Waiting ~20 s for Gazebo to load, then auto-takeoff to z=1.5 and move to x=4."
echo "   RViz shows: trail (cyan LINE_STRIP), obstacles (orange CYLINDER), AprilTag (green ARROW)"
echo "   Gazebo: translucent blue spheres mark the flight trail every 0.5 m"
echo ""
echo "   Monitor obstacle detector:"
echo "   rostopic echo /obstacle/info"
echo "     data[0]=detected  data[1]=dist(m)  data[2]=left_clear  data[3]=right_clear"
echo ""
echo "   Send a trajectory to the planner (total_time=20s in header.stamp):"
echo "   rostopic pub -1 /traj/waypoints nav_msgs/Path \\"
echo "     '{header:{stamp:{secs:20},frame_id:world},poses:[{pose:{position:{x:0,y:0,z:1.5}}},{pose:{position:{x:14,y:0,z:1.5}}}]}'"
echo "   rostopic echo /traj/status"
echo ""
echo "   Check AprilTag texture loaded:"
echo "   rostopic echo /gazebo/model_states | grep apriltag_pad"