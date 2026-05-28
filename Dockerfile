FROM ros:jazzy-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    ros-jazzy-robot-localization \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /loki_ws

# Copy source
COPY src/ src/

# Build all packages including ROS-TCP-Connector
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace on startup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /loki_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-c", \
    "source /opt/ros/jazzy/setup.bash && \
     source /loki_ws/install/setup.bash && \
     ros2 launch loki_bringup sim.launch.py"]
