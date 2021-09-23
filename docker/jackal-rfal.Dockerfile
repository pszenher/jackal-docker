ARG ROS_DISTRO
FROM pszenher/jackal:${ROS_DISTRO}

# ====================================================================
# | Source Packages Compilation/Installation                         |
# ====================================================================

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Clone source code for github packages
RUN mkdir -p /etc/ros/catkin_ws/{src,deps} && \
    # git clone --branch "4.0.2" --depth=1 "https://github.com/borglab/gtsam" "/etc/ros/catkin_ws/deps/gtsam" && \
    # git clone --depth=1 "https://github.com/pszenher/LIO-SAM" "/etc/ros/catkin_ws/src/LIO-SAM" && \
    # git clone --depth=1 "https://github.com/pszenher/BGK_traversability_mapping" "/etc/ros/catkin_ws/src/BGK_traversability_mapping" && \
    # git clone --depth=1 "https://github.com/pszenher/LeGO-LOAM" "/etc/ros/catkin_ws/src/LeGO-LOAM" && \
    # git clone --depth=1 "https://github.com/pszenher/la3dm" "/etc/ros/catkin_ws/src/la3dm" && \
    # TODO: review spin_hokuyo library dependencies to allow install
    # git clone --depth=1 "https://github.com/pszenher/spin_hokuyo" "/etc/ros/catkin_ws/src/spin_hokuyo"
    git clone --depth=1 "https://github.com/pszenher/zed_cpu_ros" "/etc/ros/catkin_ws/src/zed_cpu_ros"

# Build ROS packages from source, install ROS bringup launch files
# hadolint ignore=SC1091
RUN source /etc/ros/setup.bash && \
    # cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON "/etc/ros/catkin_ws/deps/gtsam" -B"/etc/ros/catkin_ws/deps/gtsam/build" && \
    # make -j"$(nproc)" -C "/etc/ros/catkin_ws/deps/gtsam/build" install && \
    # TODO: fix package installation configuration in each package
    # catkin config --workspace "/etc/ros/catkin_ws" --install-space "/opt/ros/${ROS_DISTRO}" --install && \
    catkin build --workspace "/etc/ros/catkin_ws"

# Switch to $JACKAL_USER user (inherited from parent image)
USER "${JACKAL_USER}"
