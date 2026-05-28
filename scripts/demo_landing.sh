#!/bin/bash
# demo_landing.sh ── Gazebo + AprilTag 精準降落
# 場景：iris 下方地面有 AprilTag，啟動視覺偵測後自動降落

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash 2>/dev/null || true

export DISPLAY=:99
# ── 同步 world 檔到 roslaunch 實際讀取的路徑 ──────────────────────────────────
# roslaunch 用 $(find rotors_gazebo)/worlds/，不是 src/worlds/
# src/worlds 是 read-only mount，所以把最新版複製過去
cp /root/catkin_ws/src/worlds/apriltag_landing.world \
   /root/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds/apriltag_landing.world

# ── 確保 AprilTag 材質在 Gazebo 系統路徑（gzserver 100% 找得到）────────────────
cp /root/catkin_ws/src/worlds/media/materials/scripts/apriltag.material \
   /usr/share/gazebo-11/media/materials/scripts/ 2>/dev/null || true
cp /root/catkin_ws/src/worlds/media/materials/textures/tag36_11_00000.png \
   /usr/share/gazebo-11/media/materials/textures/ 2>/dev/null || true
export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:${GAZEBO_MODEL_PATH}
# worlds/ 也要在 RESOURCE_PATH 裡，Gazebo 才能找到 media/materials/scripts/apriltag.material
export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/worlds:/root/catkin_ws/src/rotors_simulator/rotors_gazebo:${GAZEBO_RESOURCE_PATH}


echo "======================================"
echo " Demo 3: AprilTag Precision Landing"
echo " Geometric Controller + Visual Servo"
echo "======================================"

# Gazebo + iris + apriltag world
xterm -title "Gazebo: AprilTag Landing" -geometry 100x30+10+50 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:\$GAZEBO_MODEL_PATH
    export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo:\$GAZEBO_RESOURCE_PATH
    roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch \
      mav_name:=iris \
      world_name:=apriltag_landing 2>&1
    bash
  " &

sleep 10

# AprilTag continuous detection
xterm -title "AprilTag Detector" -geometry 80x15+10+450 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    roslaunch apriltag_ros continuous_detection.launch \
      camera_name:=/iris/vi_sensor/camera_depth \
      image_topic:=image_raw \
      tags_config:=/root/catkin_ws/src/tags.yaml \
      settings_config:=/root/catkin_ws/src/settings.yaml
    bash
  " &

sleep 3

# Landing controller node
xterm -title "Landing Controller" -geometry 80x15+10+620 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    echo 'Starting AprilTag landing controller...'
    python3 /root/scripts/apriltag_lander.py 2>&1
    bash
  " &

echo ""
echo "✅ 精準降落 demo 啟動中..."
echo "   確認 /tag_detections topic 有訊號後降落會自動開始"
echo ""
echo "   查看 tag 偵測：rostopic echo /tag_detections"
