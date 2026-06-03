#!/bin/bash
SESSION="loki"
WS="$HOME/loki_ws"
SRC="source /opt/ros/jazzy/setup.bash && source $HOME/auv_demo_ws/install/setup.bash && source $WS/install/setup.bash"

echo "Cleaning up..."
tmux kill-session -t "$SESSION" 2>/dev/null
sleep 1
pkill -9 -f "loki_controller"  2>/dev/null
pkill -9 -f "loki_monitor"     2>/dev/null
pkill -9 -f "ros_tcp_endpoint" 2>/dev/null
pkill -9 -f "foxglove_bridge"  2>/dev/null
pkill -9 -f "ekf_node"         2>/dev/null
pkill -9 -f "path_publisher"   2>/dev/null
pkill -9 -f "circle_test"      2>/dev/null
pkill -9 -f "box_test"         2>/dev/null
pkill -9 -f "ros2"             2>/dev/null
ros2 daemon stop 2>/dev/null
sleep 2
ros2 daemon start 2>/dev/null
sleep 1

echo "Building..."
cd "$WS"
source /opt/ros/jazzy/setup.bash
source "$HOME/auv_demo_ws/install/setup.bash"
colcon build --packages-select loki_control loki_monitor loki_hardware loki_hardware_imu loki_hardware_dvl loki_localization loki_description loki_msgs loki_bringup
source "$WS/install/setup.bash"
echo "Build done."
sleep 1

tmux new-session -d -s "$SESSION" -x 220 -y 50
tmux split-window -v -t "$SESSION"

# Pane 0: launch + path publisher
tmux send-keys -t "$SESSION:0.0" \
  "$SRC && ros2 launch loki_bringup sim.launch.py & sleep 8 && python3 $WS/src/loki_localization/scripts/path_publisher.py" C-m

# Pane 1: commands
tmux send-keys -t "$SESSION:0.1" "$SRC"$'\n'
tmux send-keys -t "$SESSION:0.1" '
alias arm="ros2 service call /system/arm std_srvs/srv/SetBool \"{data: true}\""
alias disarm="ros2 service call /system/arm std_srvs/srv/SetBool \"{data: false}\""
alias depth="f(){ ros2 topic pub --once /target/depth std_msgs/msg/Float64 \"{data: \$1}\"; unset -f f; }; f"
alias speed="f(){ ros2 topic pub --once /target/speed std_msgs/msg/Float64 \"{data: \$1}\"; unset -f f; }; f"
alias heading="f(){ ros2 topic pub --once /target/heading std_msgs/msg/Float64 \"{data: \$1}\"; unset -f f; }; f"
alias mass="f(){ ros2 topic pub --once /target/moving_mass std_msgs/msg/Float64 \"{data: \$1}\"; unset -f f; }; f"
# go <depth> <speed> <mass>
alias go="f(){ ros2 topic pub --once /loki/command loki_msgs/msg/LokiCommand \"{target_depth: \$1, target_speed: \$2, target_moving_mass: \$3}\"; unset -f f; }; f"
alias circle="python3 ~/loki_ws/src/loki_localization/scripts/circle_test.py"
alias box="python3 ~/loki_ws/src/loki_localization/scripts/box_test.py"
clear
' C-m

tmux set -g mouse on
tmux select-pane -t "$SESSION:0.1"
tmux attach-session -t "$SESSION"