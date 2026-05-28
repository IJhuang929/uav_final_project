#!/bin/bash
# demo_hover.sh ── Gazebo + RotorS iris hover
# 在 noVNC 的 xterm 裡執行這個腳本
# 會開啟 Gazebo 視窗，看到 iris 無人機在室內場景懸停

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash 2>/dev/null || true

export DISPLAY=:99
export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo:$GAZEBO_RESOURCE_PATH

echo "======================================"
echo " Demo 1: RotorS Hover in Gazebo"
echo " 關閉：Ctrl+C 或在各視窗輸入 exit"
echo "======================================"
echo ""

# 啟動 Gazebo + iris 無人機（室內地形場景）
xterm -title "Gazebo + Iris" -geometry 100x30+10+400 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:\$GAZEBO_MODEL_PATH
    export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo:\$GAZEBO_RESOURCE_PATH
    echo '[1/2] Launching Gazebo with Iris drone...'
    roslaunch rotors_gazebo mav_hovering_example.launch \
      mav_name:=iris world_name:=basic 2>&1
    bash
  " &

sleep 8

# 啟動幾何控制器
xterm -title "Geometric Controller" -geometry 100x20+10+700 \
  -fa 'Monospace' -fs 10 -bg '#0d1117' -fg '#c9d1d9' \
  -e bash -c "
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash 2>/dev/null || true
    export DISPLAY=:99
    echo '[2/2] Launching geometric controller...'
    roslaunch rotors_gazebo mav_with_geometric_controller.launch \
      mav_name:=iris 2>&1
    bash
  " &

echo ""
echo "✅ Gazebo 開啟中，等待約 10 秒讓模型載入..."
echo "   你應該會看到 iris 無人機懸停在場景中"
