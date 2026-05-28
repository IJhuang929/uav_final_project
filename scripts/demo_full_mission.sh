#!/bin/bash
# demo_full_mission.sh — Full autonomous mission: takeoff → explore → avoid → land
# ─────────────────────────────────────────────────────────────────────────────
# Nodes launched (in order):
#   1. Gazebo + iris + vi_sensor    (obstacle_course.world)
#   2. gazebo_controller            → /iris/odometry_sensor1/pose
#   3. apriltag_ros                 → /tag_detections
#   4. obstacle_detector            → /obstacle/info
#   5. trajectory_planner           → /iris/command/pose, /traj/status
#   6. trajectory_viz               → RViz markers
#   7. apriltag_lander              → /iris/command/velocity  (waits for enable)
#   8. RViz
#   9. Gazebo unpause               (waits for Lee controller topic)
#  10. mission_manager              (starts last — orchestrates everything)
#
# Mission sequence:
#   IDLE → TAKEOFF (z=1.5) → EXPLORE (fly toward tag at x=14)
#   → ALIGN → LAND (AprilTag precision landing)

set -e
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash 2>/dev/null || true

export DISPLAY=:99

# ── Patch iris xacro to add downward camera (idempotent) ─────────────────────
python3 /root/scripts/patch_xacro.py || true

# ── Sync world file and AprilTag assets ───────────────────────────────────────
cp /root/catkin_ws/src/worlds/obstacle_course.world \
   /root/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds/obstacle_course.world

cp /root/catkin_ws/src/worlds/media/materials/scripts/apriltag.material \
   /usr/share/gazebo-11/media/materials/scripts/ 2>/dev/null || true
cp /root/catkin_ws/src/worlds/media/materials/textures/tag36_11_00000.png \
   /usr/share/gazebo-11/media/materials/textures/ 2>/dev/null || true

export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/worlds:/root/catkin_ws/src/rotors_simulator/rotors_gazebo:${GAZEBO_RESOURCE_PATH}

echo "=============================================="
echo " Full Autonomous Mission"
echo " IDLE → TAKEOFF → EXPLORE → AVOID → LAND"
echo "=============================================="
echo ""
echo " World: obstacle_course   AprilTag @ (14, 0)"
echo " Pillars: P1(4,0) P2(6.5,1.8) P3(7,-1.5) P4(9.5,0.6) P5(11.5,-1)"
echo ""

# ── 1. Gazebo ─────────────────────────────────────────────────────────────────
xterm -title "Gazebo: Full Mission" \
  -geometry 110x30+10+10 \
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

echo "Waiting 20 s for Gazebo to load..."
sleep 20

# ── 1.5. RTAB-Map visual odometry (skip if package not installed) ─────────────
if rospack find rtabmap_odom >/dev/null 2>&1; then
  xterm -title "RTAB-Map Odometry" \
    -geometry 80x12+1400+10 \
    -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#a371f7' \
    -e bash -c "
      source /opt/ros/noetic/setup.bash
      source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
      export DISPLAY=:99
      echo '[RTAB-Map] Starting RGB-D visual odometry → /rtabmap/odom'
      roslaunch /root/scripts/rtabmap.launch 2>&1
      bash
    " &
  echo "Waiting for RTAB-Map odometry node to advertise..."
  until rostopic list 2>/dev/null | grep -q '/rtabmap/odom'; do sleep 1; done
  echo "RTAB-Map ready."
  export USE_RTABMAP=true
else
  echo "[WARN] rtabmap_odom not installed — falling back to Gazebo ground truth"
  echo "       (install: apt-get install ros-noetic-rtabmap-odom)"
  export USE_RTABMAP=false
fi

# ── 2. Gazebo pose relay ──────────────────────────────────────────────────────
xterm -title "Gazebo Controller" \
  -geometry 80x10+10+440 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#58a6ff' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    export USE_RTABMAP=${USE_RTABMAP}
    echo '[gazebo_controller] USE_RTABMAP=${USE_RTABMAP}'
    python3 /root/scripts/gazebo_controller.py 2>&1
    bash
  " &

sleep 2

# ── 3. AprilTag detector ──────────────────────────────────────────────────────
xterm -title "AprilTag Detector" \
  -geometry 80x12+10+560 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#58a6ff' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    roslaunch apriltag_ros continuous_detection.launch \
      camera_name:=/iris/cam_down \
      image_topic:=image_raw \
      tags_config:=/root/scripts/tags.yaml 2>&1
    bash
  " &

sleep 2

# ── 4. Obstacle detector ──────────────────────────────────────────────────────
xterm -title "Obstacle Detector" \
  -geometry 80x12+700+440 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#ffa657' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    python3 /root/scripts/obstacle_detector.py 2>&1
    bash
  " &

sleep 1

# ── 5. Trajectory planner ─────────────────────────────────────────────────────
xterm -title "Trajectory Planner" \
  -geometry 80x12+700+580 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#ffa657' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    python3 /root/scripts/trajectory_planner.py 2>&1
    bash
  " &

sleep 1

# ── 6. Trajectory visualisation (background) ─────────────────────────────────
xterm -title "Traj Viz" \
  -geometry 80x8+700+720 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#8b949e' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    python3 /root/scripts/trajectory_viz.py 2>&1
    bash
  " &

sleep 1

# ── 7. AprilTag precision lander (disabled until mission_manager reaches APPROACH) ──
xterm -title "AprilTag Lander" \
  -geometry 80x12+10+700 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#3fb950' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    LANDER_INITIALLY_ENABLED=false python3 /root/scripts/apriltag_lander.py 2>&1
    bash
  " &

sleep 1

# ── 8. RViz ───────────────────────────────────────────────────────────────────
xterm -title "RViz: Mission" \
  -geometry 80x24+700+10 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    rviz -d /root/scripts/trajectory_viz.rviz 2>&1
    bash
  " &

sleep 2

# ── 9. Unpause Gazebo (wait until Lee controller is ready) ────────────────────
echo "Waiting for Lee controller topic..."
until rostopic list 2>/dev/null | grep -q '/iris/command/pose'; do sleep 1; done
echo "Unpausing Gazebo..."
rosservice call /gazebo/unpause_physics || true
sleep 2

# ── 10. Mission Manager (starts last — all services must be up) ───────────────
xterm -title "★ Mission Manager ★" \
  -geometry 90x16+10+840 \
  -fa 'Monospace' -fs 11 -bg '#161b22' -fg '#e6edf3' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    echo '>>> Mission Manager starting — drone will take off in ~3 s <<<'
    python3 /root/scripts/mission_manager.py 2>&1
    bash
  " &

echo ""
echo "✅  Full mission launched."
echo ""
echo "   Monitor topics:"
echo "   rostopic echo /mission/state          # IDLE→TAKEOFF→EXPLORE→AVOID→ALIGN→LAND"
echo "   rostopic echo /obstacle/info          # [detected, dist, left_clear, right_clear]"
echo "   rostopic echo /traj/status            # IDLE / EXECUTING / DONE / FAILED"
echo "   rostopic echo /tag_detections         # AprilTag visible?"
echo ""
echo "   Expected sequence:"
echo "   1. Drone takes off to z=1.5 m"
echo "   2. Flies toward AprilTag at (14, 0) via Minimum Snap trajectory"
echo "   3. Dodges pillars if obstacle distance < 2.5 m"
echo "   4. On tag detection → ALIGN → precision landing"
