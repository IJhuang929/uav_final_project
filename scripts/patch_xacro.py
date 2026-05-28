#!/usr/bin/env python3
"""
Patch iris_with_vi_sensor.xacro to add a downward-facing camera.

Camera mounting: rpy="pi 0 0" (roll 180 deg) on base_link bottom.
  - camera z (depth) → world -z (straight down)  → tag_pos.z = altitude above tag
  - camera x         → body +x (forward)          → tag_pos.x = forward/back offset
  - camera y         → body -y (right)             → tag_pos.y = right/left offset
ROS topic: /<namespace>/cam_down/image_raw
"""
import glob
import sys

MARKER = "cam_down"

SNIPPET = """
  <!-- downward camera for AprilTag precision landing -->
  <link name="${namespace}/cam_down_link"/>
  <joint name="${namespace}/cam_down_joint" type="fixed">
    <parent link="${namespace}/base_link"/>
    <child link="${namespace}/cam_down_link"/>
    <origin xyz="0 0 -0.05" rpy="3.14159265 0 0"/>
  </joint>
  <gazebo reference="${namespace}/cam_down_link">
    <sensor type="camera" name="${namespace}_cam_down">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047198</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.01</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="${namespace}_cam_down_controller" filename="libgazebo_ros_camera.so">
        <cameraName>${namespace}/cam_down</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>${namespace}/cam_down_link</frameName>
        <updateRate>30</updateRate>
      </plugin>
    </sensor>
  </gazebo>
"""

candidates = glob.glob(
    '/root/catkin_ws/src/rotors_simulator/rotors_description/urdf/iris_with_vi_sensor.xacro'
)
if not candidates:
    print("ERROR: iris_with_vi_sensor.xacro not found", file=sys.stderr)
    sys.exit(1)

path = candidates[0]
text = open(path).read()

if MARKER in text:
    print(f"Already patched: {path}")
    sys.exit(0)

text = text.replace('</robot>', SNIPPET + '</robot>')
open(path, 'w').write(text)
print(f"Patched: {path}")
