## data pipeline
```
   /imu/data         /depth/pose      /dvl/twist_stamped
                          │
                          ▼
┌───────────────────────────────────────────────────────┐
│                   loki_localization                   │
│                 EKF → /odometry/filtered              │
└──────────────────────────┬────────────────────────────┘
                           ▼
┌───────────────────────────────────────────────────────┐
│                     loki_control                      │
│                                                       │
│       Speed  → PI  → /cmd/thruster    (PWM)           │
│       Yaw    → PI  → /cmd/rudder      (PWM)           │
│       Depth  → PI  → desired_pitch    (Deg)           │
│       Pitch  → P   → /cmd/elevator    (PWM)           │
│       Mass         → /cmd/moving_mass (m)             │
└──────────────────────────┬────────────────────────────┘
                           ▼
┌───────────────────────────────────────────────────────┐
│                     loki_hardware                     │
│                     hw_bridge.py                      │
└──────────────────────────┬────────────────────────────┘
                           ▼
┌───────────────────────────────────────────────────────┐
│                       Actuators                       │
└───────────────────────────────────────────────────────┘
```

## control design

### overview
The controller receives information of its current state from the EKF output (/odometry/filtered). 
The axes (speed, yaw, depth) are treated as independent and a decoupled PID is used, on the assumption that cross coupling effects are negligible at lower speeds. The 1-D PID implementation is adapted from Rafael Pérez Seguí. The controller outputs PWM signals to the respective actuators to drive towards the desired state. 

### cascading
Depth and pitch are physically dependent, tuning them separately results in conflict (chicken and egg problem).
The depth axis uses a cascaded PID, an outer depth loop and an inner pitch loop.
The outer loop runs at 20hz and outputs the desired pitch angle while the inner loop runs at 100hz, driving the elevator to achieve the desired pitch. The rate difference allows for smoother depth response as compared to overcompensation if both loops run at the same frequency. The outer loop output is restricted to +- 15 degrees as a physical safety limit but can be adjusted when testing.

### anti-windup
The integrator term accumulates over time to eliminate steady state error. We implemented anti-windup to prevent the integral error from growing beyond a set limit regardless of how long the error persists.

### watchdog
If no odom data is recieved by the outer loop for >2s, is_armed is set to false (disarm). The inner loop checks is_armed every 10ms and sets all actuators to 1500 PWM and shifts moving mass to neutral position. This is a safety feature to protect against EKF crashes or failure in sensors. 



## commands

There are 2 ways to send commands to the controller:
1. individual topics (/target/depth, /target/heading, /target/speed, /target/moving_mass) separately
2. LokiCommand, single message that sets target depth speed and moving mass simultaneously (for results to be replicable)
