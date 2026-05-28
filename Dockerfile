FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ── Base ──────────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    locales tzdata curl wget git sudo gnupg2 lsb-release \
    software-properties-common ca-certificates \
    && locale-gen en_US.UTF-8 \
    && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && rm -rf /var/lib/apt/lists/*

# ── ROS Noetic ────────────────────────────────────────────────────────────────
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros-latest.list' \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator \
    python3-wstool build-essential python3-catkin-tools \
    python3-pip python3-numpy \
    && rosdep init && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# ── Gazebo 11 + rendering libs ────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    gazebo11 libgazebo11-dev \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ── RotorS dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-noetic-joy ros-noetic-octomap-ros \
    ros-noetic-mavros-msgs \
    ros-noetic-geographic-msgs ros-noetic-control-toolbox \
    ros-noetic-transmission-interface ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers ros-noetic-position-controllers \
    ros-noetic-robot-state-publisher ros-noetic-xacro \
    libgoogle-glog-dev libgflags-dev liblapack-dev libsuitesparse-dev \
    libfmt-dev python3-wstool python3-catkin-tools \
    protobuf-compiler libprotobuf-dev \
    python3-cvxopt python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# ── Phase2 Python dependencies ────────────────────────────────────────────────
RUN python3 -m pip install --upgrade pip && python3 -m pip install cvxpy

# ── AprilTag + vision + RTAB-Map odometry ────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-noetic-apriltag-ros \
    ros-noetic-image-transport ros-noetic-cv-bridge \
    ros-noetic-tf2-ros ros-noetic-imu-filter-madgwick \
    ros-noetic-robot-localization \
    ros-noetic-rtabmap-odom \
    && rm -rf /var/lib/apt/lists/*

# ── Virtual display + noVNC ───────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    xvfb x11vnc openbox xterm supervisor dbus-x11 \
    xfonts-base x11-xserver-utils xdotool \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/novnc/noVNC.git /opt/novnc \
 && git clone --depth 1 https://github.com/novnc/websockify.git /opt/novnc/utils/websockify \
 && ln -s /opt/novnc/vnc.html /opt/novnc/index.html

# ── Build catkin workspace ────────────────────────────────────────────────────
WORKDIR /root/catkin_ws/src

# RotorS simulator
RUN git clone --depth 1 https://github.com/ethz-asl/rotors_simulator.git

# mav_trajectory_generation (Minimum Snap)
RUN git clone --depth 1 https://github.com/ethz-asl/mav_trajectory_generation.git
RUN git clone --depth 1 https://github.com/ethz-asl/eigen_catkin.git
RUN git clone --depth 1 https://github.com/ethz-asl/glog_catkin.git
RUN git clone --depth 1 https://github.com/catkin/catkin_simple.git
RUN git clone --depth 1 https://github.com/ethz-asl/nlopt.git
RUN git clone --depth 1 https://github.com/ethz-asl/mav_comm.git
RUN git clone --depth 1 https://github.com/ethz-asl/eigen_checks.git
RUN git clone --depth 1 https://github.com/ethz-asl/gflags_catkin.git

WORKDIR /root/catkin_ws

RUN apt-get update \
 && bash -c "source /opt/ros/noetic/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic" \
 && rm -rf /var/lib/apt/lists/*

RUN bash -c "source /opt/ros/noetic/setup.bash \
    && catkin config --extend /opt/ros/noetic \
    && catkin build -DCMAKE_BUILD_TYPE=Release -j$(nproc) --summarize \
    "

# ── Shell env ─────────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "source /root/catkin_ws/devel/setup.bash 2>/dev/null || true" >> /root/.bashrc \
 && echo 'export GAZEBO_MODEL_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo/models:$GAZEBO_MODEL_PATH' >> /root/.bashrc \
 && echo 'export GAZEBO_RESOURCE_PATH=/root/catkin_ws/src/rotors_simulator/rotors_gazebo:$GAZEBO_RESOURCE_PATH' >> /root/.bashrc

# ── Copy runtime files ────────────────────────────────────────────────────────
COPY config/supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY config/openbox-rc.xml   /root/.config/openbox/rc.xml
COPY config/openbox-menu.xml /root/.config/openbox/menu.xml
COPY scripts/                /root/scripts/
RUN chmod +x /root/scripts/*.sh /root/scripts/*.py 2>/dev/null || true

COPY worlds/ /root/catkin_ws/src/worlds/

# ── Integrate custom world + AprilTag texture into RotorS paths ───────────────
# 1. Download tag36h11 id=0 PNG from the official apriltag-imgs repository
# 2. Copy world files → rotors_gazebo/worlds/ (so world_name:=obstacle_course works)
# 3. Copy Ogre material script + texture → rotors_gazebo media dir
RUN mkdir -p \
      /root/catkin_ws/src/worlds/media/materials/textures \
      /root/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds \
      /root/catkin_ws/src/rotors_simulator/rotors_gazebo/media/materials/scripts \
      /root/catkin_ws/src/rotors_simulator/rotors_gazebo/media/materials/textures \
 && wget -q \
      "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00000.png" \
      -O /root/catkin_ws/src/worlds/media/materials/textures/tag36_11_00000.png \
 && cp /root/catkin_ws/src/worlds/*.world \
       /root/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds/ \
 && cp /root/catkin_ws/src/worlds/media/materials/scripts/* \
       /root/catkin_ws/src/rotors_simulator/rotors_gazebo/media/materials/scripts/ \
 && cp /root/catkin_ws/src/worlds/media/materials/textures/tag36_11_00000.png \
       /root/catkin_ws/src/rotors_simulator/rotors_gazebo/media/materials/textures/ \
 && cp /root/catkin_ws/src/worlds/media/materials/scripts/* \
       /usr/share/gazebo-11/media/materials/scripts/ \
 && cp /root/catkin_ws/src/worlds/media/materials/textures/tag36_11_00000.png \
       /usr/share/gazebo-11/media/materials/textures/

EXPOSE 8080 5900 11311

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
