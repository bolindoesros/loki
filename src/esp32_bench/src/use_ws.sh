#!/bin/bash
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$WS_DIR/install/microros_msgs:$WS_DIR/install/esp32_bench:$AMENT_PREFIX_PATH"
export CMAKE_PREFIX_PATH="$WS_DIR/install/microros_msgs:$WS_DIR/install/esp32_bench:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$WS_DIR/install/microros_msgs/lib:$WS_DIR/install/esp32_bench/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$WS_DIR/install/microros_msgs/local/lib/python3.10/dist-packages:$WS_DIR/install/esp32_bench/local/lib/python3.10/dist-packages:$PYTHONPATH"
