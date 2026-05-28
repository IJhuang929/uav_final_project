#!/bin/bash
set -e

RESOLUTION="${RESOLUTION:-1920x1080}"
export RESOLUTION

mkdir -p /var/log/supervisor /root/.config/openbox /root/.gazebo

cat <<'EOF'
╔══════════════════════════════════════════════════════╗
║         UAV Gazebo Simulation (RotorS + ROS)         ║
╠══════════════════════════════════════════════════════╣
║  noVNC  →  http://localhost:8080/vnc.html            ║
║  VNC    →  localhost:5900  (no password)             ║
╚══════════════════════════════════════════════════════╝
EOF

echo "Resolution: $RESOLUTION"
echo "Starting supervisord..."

exec /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
