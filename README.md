# loki auv workspace

## packages

| package | role |
|---|---|
| `loki_bringup` | launch files |
| `loki_control` | cascaded PID — speed, yaw, depth, pitch |
| `loki_description` | URDF + meshes |
| `loki_hardware` | ESP32 bridge → `/pc_to_esp_cmd` |
| `loki_hardware_imu` | IMU/mag/pressure driver |
| `loki_hardware_dvl` | Cerulean Tracker 650 DVL driver |
| `loki_localization` | EKF via `robot_localization` |
| `loki_monitor` | republishes state to `/monitor/*` for Foxglove |
| `loki_msgs` | custom message types |

## launch

```bash
ros2 launch loki_bringup real.launch.py   # real robot
ros2 launch loki_bringup sim.launch.py    # unity sim
```

## arm / disarm

```bash
ros2 service call /system/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /system/arm std_srvs/srv/SetBool "{data: false}"
```

## commands

```bash
ros2 topic pub /target/depth       std_msgs/msg/Float64 "{data: 1.0}"   # metres
ros2 topic pub /target/heading     std_msgs/msg/Float64 "{data: 90.0}"  # degrees 0-360
ros2 topic pub /target/speed       std_msgs/msg/Float64 "{data: 0.5}"   # m/s
ros2 topic pub /target/moving_mass std_msgs/msg/Float64 "{data: 0.5}"   # 0.0-1.0
```

## pid tuning

config: `loki_control/config/pid_params.yaml`

| loop | kp | ki | kd | output |
|---|---|---|---|---|
| speed | 1200 | 300 | 0 | ±400 |
| yaw | -30 | -5 | 0 | ±400 |
| depth (outer) | 120 | 0 | 0 | -40 to +20 |
| pitch (inner) | 150 | 0 | 0 | ±400 |

rebuild after changes: `colcon build && source install/setup.bash`

## foxglove

connect to `ws://ROBOT_IP:8765` — all state on `/monitor/*`

## build

```bash
cd ~/loki_ws && colcon build && source install/setup.bash
```
