# Smart Mobility Hub development image (M5 infrastructure).
#
# ROS 2 Jazzy on Ubuntu 24.04. The base tag is a multi-arch index, so the same
# file builds natively on Apple Silicon (linux/arm64/v8) and on amd64 PCs.
#
# Derived from M1-B's Dockerfile (branch feature/m1b-demand-predictor): same
# base image and the same Python packages, so S2 code also runs here. One
# difference: the source is NOT copied into the image. docker/hub.sh mounts
# the repository at /ws, so editing code never requires an image rebuild.
#
# The base is pinned by digest (checked 2026-10-20) so every teammate builds
# on the identical ROS installation. Update the digest deliberately.
FROM ros:jazzy-ros-base@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy python3-sklearn python3-matplotlib python3-pytest \
        python3-yaml \
        ros-jazzy-launch-testing ros-jazzy-launch-testing-ros \
        ros-jazzy-ament-flake8 ros-jazzy-ament-pep257 \
    && rm -rf /var/lib/apt/lists/*

# Keep DDS traffic inside the container; tests never leak onto the LAN.
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

WORKDIR /ws
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
