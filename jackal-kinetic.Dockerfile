FROM ubuntu:xenial-20210416

ENV ROS_DISTRO=kinetic

ARG JACKAL_USER="rfal"
ARG JACKAL_PASSWORD='$6$lTlXs59jEbj2Je$zZ/eQ5S142JWYrglIpJq8NtDo8uKkIu6WkTcucVWy.7uRk2O/1DOQMMWgLi6/Lkvm1OlkPnKpRFrVXGRM5rNG0'

ARG ROS_URL="https://mirrors.osuosl.org/pub/ros/packages.ros.org"
ARG CLEARPATH_URL="https://packages.clearpathrobotics.com"

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install boilerplate utilities
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update && apt-get install --no-install-recommends -y \
    \
    # System packages
    build-essential=12.1ubuntu2 \
    ca-certificates=20210119~16.04.1 \
    curl=7.47.0-1ubuntu2.19 \
    dhcpcd5=6.10.1-1 \
    software-properties-common=0.96.20.10 \
    \
    # Package management packages
    apt-transport-https=1.2.35 \
    python-pip=8.1.1-2ubuntu0.6 \
    python3-pip=8.1.1-2ubuntu0.6 \
    \
    # Linux kernel/init packages
    initramfs-tools=0.122ubuntu8.17 \
    linux-image-generic=4.4.0.210.216 \
    systemd-sysv=229-4ubuntu21.31 \
    \
    # Sysadmin tool packages
    htop=2.0.1-1ubuntu1 \
    less=481-2.1ubuntu0.2 \
    openssh-server=1:7.2p2-4ubuntu2.10 \
    screen=4.3.1-2ubuntu0.1 \
    vim=2:7.4.1689-3ubuntu1.5 \
    wicd-curses=1.7.4+tb2-1 \
    \
    && rm -rf /var/lib/apt/lists/*

# Add ROS deb package repo
RUN echo "deb ${ROS_URL}/ros/ubuntu/ xenial main" > /etc/apt/sources.list.d/ros-latest.list &&\
    curl -sSL "${ROS_URL}/ros.key" | apt-key add -

# Add Clearpath deb package repo
RUN echo "deb ${CLEARPATH_URL}/stable/ubuntu xenial main" > /etc/apt/sources.list.d/clearpath-latest.list &&\
    curl -sSL "${CLEARPATH_URL}/public.key" | apt-key add -

# Install deb packages from newly installed repos
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update && apt-get install --no-install-recommends -y \
    \
    # Jackal ROS Packages
    python-catkin-tools=0.6.1-1 \
    python-rosdep=0.20.1-1 \
    python-rosinstall=0.7.8-1 \
    python-rosinstall-generator=0.1.22-1 \
    python-wstool=0.1.17-1 \
    ros-kinetic-robot=1.3.2-0xenial-20201103-233358+0000 \
    ros-kinetic-ros-base=1.3.2-0xenial-20201103-121012+0000 \
    \
    # Jackal Clearpath Packages
    python-ds4drv=0.5.2xenial \
    ros-kinetic-jackal-robot=0.5.1-1xenial-20210310-162855-0500 \
    \
    # Dependencies: la3dm
    ros-kinetic-octomap-mapping \
    \
    && rm -rf /var/lib/apt/lists/*

# Merge system folder config files into root filesystem
COPY ./root-filesystem/. /

# Add ROS_DISTRO env var at insertion point in global setup.bash
RUN sed -i '/^######$/i ROS_DISTRO='"${ROS_DISTRO}" /etc/ros/setup.bash

# Initialize system-wide rosdep
RUN rosdep init

# Clone source code for github packages
RUN mkdir -p /etc/ros/catkin_ws/{src,deps} && \
    git clone --branch "4.0.2" "https://github.com/borglab/gtsam" "/etc/ros/catkin_ws/deps/gtsam" && \
    git clone "https://github.com/pszenher/LIO-SAM" "/etc/ros/catkin_ws/src/LIO-SAM" && \
    git clone "https://github.com/RobustFieldAutonomyLab/BGK_traversability_mapping" "/etc/ros/catkin_ws/src/BGK_traversability_mapping" && \
    git clone "https://github.com/RobustFieldAutonomyLab/LeGO-LOAM" "/etc/ros/catkin_ws/src/LeGO-LOAM" && \
    git clone "https://github.com/RobustFieldAutonomyLab/la3dm" "/etc/ros/catkin_ws/src/la3dm"

# Build ROS packages from source, install ROS bringup launch files
# hadolint ignore=SC1091
RUN source /etc/ros/setup.bash && \
    cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON "/etc/ros/catkin_ws/deps/gtsam" -B"/etc/ros/catkin_ws/deps/gtsam/build" && \
    make -j4 -C "/etc/ros/catkin_ws/deps/gtsam/build" install && \
    catkin config --workspace "/etc/ros/catkin_ws" --install-space "/opt/ros/${ROS_DISTRO}" --install && \
    catkin build --workspace "/etc/ros/catkin_ws" && \
    /etc/ros/install-ros-bringup.py

# Add $JACKAL_USER user with group memberships and hashed password
RUN useradd -mUG sudo "${JACKAL_USER}" && \
    echo "${JACKAL_USER}:${JACKAL_PASSWORD}" \
    | chpasswd -e

# Copy home directory contents into $JACKAL_USER home dir and set ownership
COPY ./home-directory/. "/home/${JACKAL_USER}"
RUN chown -R "${JACKAL_USER}:${JACKAL_USER}" "/home/${JACKAL_USER}"

# Switch to $JACKAL_USER user
USER "${JACKAL_USER}"

# Update rosdep as rfal user
RUN rosdep update

# Build home directory catkin workspace
# hadolint ignore=SC1091
RUN source /etc/ros/setup.bash && \
    mkdir -p "/home/${JACKAL_USER}/catkin_ws/src" && \
    catkin build --workspace "/home/${JACKAL_USER}/catkin_ws"

# Set working directory to user home dir
WORKDIR "/home/${JACKAL_USER}"

# Set default command to bash
CMD ["/bin/bash"]
