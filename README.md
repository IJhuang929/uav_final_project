# UAV Gazebo Simulation
### RotorS + Minimum Snap + AprilTag 精準降落 + noVNC

---

## 快速開始

```bash
# 1. Build（第一次約 20 分鐘）
docker compose build

# 2. 啟動
docker compose up -d

# 3. 開瀏覽器
open http://localhost:8080/vnc.html
```

桌面出來後，**右鍵點桌面** 可以直接選 demo 啟動。

---

## 三個 Demo

### Demo 1 — Gazebo + 懸停
```bash
bash /root/scripts/demo_hover.sh
```
Gazebo 視窗裡看到 iris 無人機懸停在場景中，幾何控制器保持穩定。

### Demo 2 — 障礙物避障（Minimum Snap 重規劃）
```bash
bash /root/scripts/demo_avoid.sh
```
Firefly 無人機在障礙物場景飛行，偵測到障礙物後 Minimum Snap 重新計算平滑路徑。

### Demo 3 — AprilTag 精準降落
```bash
bash /root/scripts/demo_landing.sh
```
Iris 下方地面有 AprilTag，視覺偵測後自動置中並精準降落。

---

## 系統架構

```
noVNC (browser)
  └── websockify → x11vnc :5900 → Xvfb :99
                                    ├── Openbox
                                    ├── Gazebo 11
                                    │     └── RotorS iris/firefly
                                    └── ROS Noetic
                                          ├── rotors_simulator
                                          ├── mav_trajectory_generation
                                          ├── apriltag_ros
                                          └── apriltag_lander.py
```

---

## 常用指令

```bash
# 進 container shell
docker compose exec uav-sim bash

# 看所有服務狀態
docker compose exec uav-sim supervisorctl status

# 重啟 roscore（掛掉時）
docker compose exec uav-sim supervisorctl restart roscore

# 看 ROS topics
docker compose exec uav-sim bash -c \
  "source /opt/ros/noetic/setup.bash && rostopic list"

# 監看 AprilTag 偵測
docker compose exec uav-sim bash -c \
  "source /opt/ros/noetic/setup.bash && rostopic echo /tag_detections"

# 看 log
docker compose logs -f uav-sim
```

---

### catkin build 失敗
```bash
docker compose exec uav-sim bash
cd /root/catkin_ws
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | grep -E "ERROR|error"
```

### RotorS launch 找不到
```bash
rospack find rotors_gazebo   # 應該要回傳路徑
source /root/catkin_ws/devel/setup.bash
```

---

## 目錄結構

```
uav_sim/
├── Dockerfile
├── docker-compose.yml
├── config/
│   ├── supervisord.conf    # 管理 Xvfb/VNC/ROS/xterm
│   ├── openbox-rc.xml      # 視窗管理員
│   └── openbox-menu.xml    # 右鍵選單（含 demo 快捷）
├── scripts/
│   ├── entrypoint.sh
│   ├── demo_hover.sh       # Demo 1: Gazebo 懸停
│   ├── demo_avoid.sh       # Demo 2: Minimum Snap 避障
│   ├── demo_landing.sh     # Demo 3: AprilTag 降落
│   └── apriltag_lander.py  # 精準降落節點（狀態機）
└── worlds/
    └── apriltag_landing.world  # 含障礙物 + AprilTag 的場景
```

ROS 專案 Debug 與重構指令】
我目前有一個基於 ROS Noetic 與 RotorS 的無人機專案，包含 mission_manager.py、apriltag_lander.py 等檔案。目前遇到以下幾個嚴重的架構與邏輯問題，請幫我一次性檢查並修復這些檔案：
Topic 搶佔問題：確認這些 apriltag_lander.py、mission_manager.py、trajectory_planner.py 發布 topic 是不是有重複的 pose，會不會讓控制器拿到錯誤的指令？
狀態機覆蓋問題：目前的狀態機會遇到進入 IDLE 就出不來的問題，幫我檢查這是為什麼？
型別報錯：obstacle_detector.py 會出現型別不對的問題 list[] 或 tuple[]，幫我解決。目前版本用的 python3.8 所以沒有這些型別。



# 規劃
起飛到 x=14 (軌跡規劃控制):
  mission_manager ──waypoints──→ trajectory_planner ──/iris/command/pose──→ Lee

到達 AprilTag 附近 (視覺伺服控制):
  mission_manager → _enable_lander(True)
  apriltag_lander ──/iris/command/pose──→ Lee  (trajectory_planner 被 cancel)
