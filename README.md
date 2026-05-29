# UAV Gazebo Simulation
### RotorS + Minimum Snap + Convex Avoidance + AprilTag 精準降落

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

## Demo

### 完整自主任務（主要 Demo）
```bash
bash /root/scripts/demo_full_mission.sh [world_name]
# 預設 world: obstacle_course_v2
```

任務序列：`IDLE → TAKEOFF → EXPLORE → OBSTACLE_REPLAN → APPROACH → LAND`

| 步驟 | 說明 |
|---|---|
| TAKEOFF | 爬升至 z = 1.5 m |
| EXPLORE | Minimum Snap 軌跡飛向 AprilTag (x=14) |
| OBSTACLE_REPLAN | 偵測到柱子 → Convex Optimization 繞障 |
| APPROACH | 前向相機鎖定 AprilTag → 飛向目標 |
| LAND | 下視相機視覺伺服 → 精準降落 |

監看 topic：
```bash
rostopic echo /mission/state     # 狀態機狀態
rostopic echo /obstacle/info     # [detected, dist, left_clear, right_clear, gap_y, gap_w]
rostopic echo /traj/status       # IDLE / EXECUTING / DONE / FAILED
rostopic echo /tag_detections    # 前向 AprilTag
```

### 其他 Demo
```bash
bash /root/scripts/demo_hover.sh          # iris 懸停
bash /root/scripts/demo_landing.sh        # AprilTag 精準降落（無障礙物）
bash /root/scripts/demo_avoid.sh          # 純避障（無降落）
bash /root/scripts/demo_obstacle_world.sh # obstacle_course v1
```

---

## 系統架構

```
noVNC (browser :8080)
  └── websockify → x11vnc :5900 → Xvfb :99
                                    ├── Openbox
                                    ├── Gazebo 11  ← obstacle_course_v2.world
                                    │     └── iris + vi_sensor + camera_down
                                    └── ROS Noetic
                                          ├── mission_manager        FSM 主控
                                          ├── trajectory_planner     Minimum Snap QP
                                          ├── obstacle_detector      PointCloud2 → 障礙資訊
                                          ├── convex_path_optimizer  繞障路徑點生成
                                          ├── apriltag_lander        下視相機精準降落
                                          ├── apriltag_ros (×2)      前向 + 下視偵測
                                          ├── gazebo_controller      位姿中繼
                                          ├── trajectory_viz         RViz 視覺化
                                          └── rtabmap_odom (選用)    視覺里程計
```

### 控制流程

```
起飛 / EXPLORE 階段（軌跡控制）：
  mission_manager ──/traj/waypoints──→ trajectory_planner ──/iris/command/pose──→ Lee Controller

APPROACH / LAND 階段（視覺伺服）：
  mission_manager → _enable_lander(True)
  apriltag_lander ──/iris/command/pose──→ Lee Controller
  （trajectory_planner 已被 cancel）
```

### 場景座標 (obstacle_course_v2)

```
Iris 起點:  (0, 0, 0.1)  → 起飛至 z = 1.5 m
柱子:  P1(4, 0)  P2(6.5, 1.8)  P3(7, -1.5)  P4(9.5, 0.6)  P5(11.5, -1)
AprilTag:  (14, 0)
走廊:  X[-1, 16]  Y[-5, 5]  Z[0, 4]
```

---

## 目錄結構

```
uav_sim/
├── Dockerfile
├── docker-compose.yml
├── config/
│   ├── supervisord.conf         # 管理 Xvfb / VNC / ROS / xterm
│   ├── openbox-rc.xml           # 視窗管理員
│   └── openbox-menu.xml         # 右鍵選單（含 demo 快捷）
├── scripts/
│   ├── entrypoint.sh
│   ├── demo_full_mission.sh     # ★ 主要 Demo：完整自主任務
│   ├── demo_hover.sh            # 懸停
│   ├── demo_landing.sh          # AprilTag 降落
│   ├── demo_avoid.sh            # 避障
│   ├── demo_obstacle_world.sh   # obstacle_course v1
│   ├── mission_manager.py       # FSM：IDLE→TAKEOFF→EXPLORE→APPROACH→LAND
│   ├── trajectory_planner.py    # Minimum Snap QP（cvxopt）
│   ├── convex_path_optimizer.py # 繞障路徑點（cvxpy OSQP）
│   ├── obstacle_detector.py     # PointCloud2 → /obstacle/info
│   ├── apriltag_lander.py       # 下視相機精準降落狀態機
│   ├── gazebo_controller.py     # Gazebo 位姿 → /iris/odometry
│   ├── trajectory_viz.py        # RViz marker 發布
│   ├── trajectory_viz.rviz      # RViz 設定（含下視相機 panel）
│   ├── iris_down_cam.launch     # iris + vi_sensor + 下視相機
│   ├── mav_with_down_cam.gazebo # 下視相機 xacro
│   ├── rtabmap.launch           # RTAB-Map 視覺里程計（選用）
│   └── tags.yaml                # AprilTag id=0 定義（size=0.8m）
└── worlds/
    ├── obstacle_course_v2.world  # ★ 主場景：走廊 + 5 柱 + AprilTag
    ├── obstacle_course.world     # v1（開放空間）
    ├── apriltag_landing.world    # 純降落場景
    └── media/
        └── materials/
            ├── scripts/apriltag.material
            └── textures/tag36_11_00000.png
```

---

## 常用指令

```bash
# 進 container shell
docker compose exec uav_sim bash

# 看服務狀態
docker compose exec uav_sim supervisorctl status

# 重啟 roscore（掛掉時）
docker compose exec uav_sim supervisorctl restart roscore

# 列出 ROS topics
docker compose exec uav_sim bash -c \
  "source /opt/ros/noetic/setup.bash && rostopic list"

# 看 log
docker compose logs -f uav_sim

# 重跑新的 demo
pkill -f xterm
sleep 2
bash /root/scripts/demo_full_mission.sh
```

---

## 疑難排解

### Gazebo 載入慢 / 模型找不到
```bash
# 確認 GAZEBO_MODEL_PATH 包含 rotors 模型
export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:$GAZEBO_MODEL_PATH
```

### catkin build 失敗
```bash
docker compose exec uav_sim bash
cd /root/catkin_ws
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | grep -E "ERROR|error"
```

### RotorS launch 找不到
```bash
rospack find rotors_gazebo   # 應回傳路徑
source /root/catkin_ws/devel/setup.bash
```

### ConvexOpt 無法使用（cvxpy 未安裝）
系統會自動 fallback 到幾何避障（geometric dodge），任務仍可完成。
若要啟用完整凸優化：
```bash
pip install cvxpy
```
