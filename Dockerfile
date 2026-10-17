# ROS 2 Jazzy (Ubuntu 24.04) image for the S2 demand predictor.
# Works on Apple Silicon (arm64) and Intel/AMD (amd64) via Docker Desktop.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy python3-sklearn python3-matplotlib python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /ws/src
RUN . /opt/ros/jazzy/setup.sh && colcon build --event-handlers console_direct-

# Every shell (including `docker exec ... bash`) gets ROS + the workspace.
RUN echo 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
