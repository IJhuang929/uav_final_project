#!/bin/bash
# demo_avoid.sh ── Gazebo + RotorS + Minimum Snap 避障
# 場景：iris 在障礙物場景中，偵測到障礙物後重新計算平滑路徑

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash 2>/dev/null || true

export DISPLAY=:99
export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo:$GAZEBO_RESOURCE_PATH

echo "======================================"
echo " Demo 2: Obstacle Avoidance"
echo " Minimum Snap Trajectory Replanning"
echo "======================================"
echo ""

# Gazebo + 障礙物場景
xterm -title "Gazebo: Obstacle World" -geometry 100x30+10+50 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:\$GAZEBO_MODEL_PATH
    export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo:\$GAZEBO_RESOURCE_PATH
    roslaunch rotors_gazebo mav_hovering_example.launch \
      mav_name:=firefly world_name:=powerplant 2>&1
    bash
  " &

sleep 10

# Minimum Snap trajectory node
xterm -title "Minimum Snap Planner" -geometry 100x20+10+450 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    echo 'Starting Minimum Snap trajectory node...'
    # Publish a waypoint path, trajectory generation node will smooth it
    roslaunch mav_trajectory_generation_ros trajectory_sampling.launch \
      mav_name:=firefly 2>&1
    bash
  " &

sleep 3

# Rviz for trajectory visualization
xterm -title "Rviz: Trajectory" -geometry 100x15+10+720 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    rviz -d /root/catkin_ws/src/mav_trajectory_generation/mav_trajectory_generation_ros/rviz/trajectory.rviz 2>&1
    bash
  " &

echo ""
echo "✅ 障礙物避障場景啟動中..."
echo "   Gazebo: 3D 物理模擬視窗"
echo "   Rviz:   軌跡可視化（Minimum Snap 路徑）"
