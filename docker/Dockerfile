# ============================================================
# ROS2 Jazzy — Course Environment
# Webots runs natively on the host OS.
# This container handles all ROS2 work only.
# Platform: linux/amd64 (consistent across all host OSes)
# ============================================================

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ── ROS2 packages ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    # Core ROS2 tools
    ros-jazzy-ros-base \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    # Webots-ROS2 bridge (driver that talks to Webots on host)
    ros-jazzy-webots-ros2 \
    # DDS middleware
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rmw-fastrtps-cpp \
    # Visualisation & debugging
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-topic \
    ros-jazzy-rqt-tf-tree \
    # TF2
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    # Navigation (useful for later assignments)
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav-msgs \
    # Common message types
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    # ros2 pi_trees
    ros-jazzy-py-trees \
    ros-jazzy-py-trees-ros-interfaces \
    ros-jazzy-py-trees-ros \
    # Build tools
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    # Utilities
    git \
    curl \
    wget \
    nano \
    vim \
    tree \
    bash-completion \
    iproute2 \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages ───────────────────────────────────────────
RUN pip install --break-system-packages \
    transforms3d \
    numpy \
    matplotlib

# ── colcon mixin (faster builds) ─────────────────────────────
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default || true

# ── rosdep init ───────────────────────────────────────────────
RUN rosdep update

# ── Environment ──────────────────────────────────────────────
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Auto-source ROS2 and the workspace in every shell
RUN echo "" >> /root/.bashrc && \
    echo "# ROS2 setup" >> /root/.bashrc && \
    echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# Helpful aliases" >> /root/.bashrc && \
    echo "alias cb='cd /ros2_ws && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias cbs='cd /ros2_ws && colcon build --symlink-install --packages-select'" >> /root/.bashrc && \
    echo "alias src='source /ros2_ws/install/setup.bash'" >> /root/.bashrc && \
    echo "alias rt='ros2 topic list'" >> /root/.bashrc && \
    echo "alias rn='ros2 node list'" >> /root/.bashrc

# ── Workspace ────────────────────────────────────────────────
WORKDIR /ros2_ws

CMD ["/bin/bash"]
