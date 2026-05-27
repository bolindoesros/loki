# LokiCommand.msg
# High-level setpoints for the Loki AUV.

std_msgs/Header header

float64 target_depth        # [m]     positive = deeper
float64 target_heading      # [deg]   0-360 NED
float64 target_speed        # [m/s]   forward
float64 target_moving_mass  # [0.0-1.0] 0=full aft, 1=full forward
