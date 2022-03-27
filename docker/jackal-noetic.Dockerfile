FROM ubuntu:focal-20210416
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ focal main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ focal-security main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ focal-updates main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ focal-backports main universe multiverse restricted"
# apt-pin: repo="deb https://mirrors.osuosl.org/pub/ros/packages.ros.org/ros/ubuntu focal main"
# apt-pin: repo="deb https://packages.clearpathrobotics.com/stable/ubuntu focal main"

ENV ROS_DISTRO=noetic

ARG ROS_URL="https://mirrors.osuosl.org/pub/ros/packages.ros.org"
ARG CLEARPATH_URL="https://packages.clearpathrobotics.com"
ARG DEBIAN_FRONTEND="noninteractive"

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ====================================================================
# | Package Management and Configuration                             |
# ====================================================================

# Install boilerplate utilities
RUN apt-get update && apt-get install --no-install-recommends -y \
    \
    # System packages
    build-essential=12.8ubuntu1.1 \
    ca-certificates=20210119~20.04.2 \
    curl=7.68.0-1ubuntu2.7 \
    dhcpcd5=7.1.0-2build1 \
    gnupg=2.2.19-3ubuntu2.1 \
    python-is-python3=3.8.2-4 \
    software-properties-common=0.99.9.8 \
    \
    # Package management packages
    apt-transport-https=2.0.6 \
    python3-pip=20.0.2-5ubuntu1.6 \
    \
    # Linux kernel/init packages
    initramfs-tools=0.136ubuntu6.7 \
    linux-image-generic=5.4.0.105.109 \
    systemd-sysv=245.4-4ubuntu3.15 \
    \
    # Sysadmin tool packages
    git=1:2.25.1-1ubuntu3.2 \
    htop=2.2.0-2build1 \
    less=551-1ubuntu0.1 \
    openssh-server=1:8.2p1-4ubuntu0.4 \
    screen=4.8.0-1ubuntu0.1 \
    vim=2:8.1.2269-1ubuntu5.7 \
    \
    && rm -rf /var/lib/apt/lists/*

# Add ROS and Clearpath deb package repos
RUN echo "deb ${ROS_URL}/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros-latest.list &&\
    curl -sSL "${ROS_URL}/ros.key" | apt-key add - \
    && \
    echo "deb ${CLEARPATH_URL}/stable/ubuntu focal main" > /etc/apt/sources.list.d/clearpath-latest.list &&\
    curl -sSL "${CLEARPATH_URL}/public.key" | apt-key add -

# Install deb packages from newly installed repos
RUN apt-get update && apt-get install --no-install-recommends -y \
    \
    # Jackal ROS Packages
    python3-catkin-tools=0.8.5-1 \
    python3-osrf-pycommon=2.0.1-1 \
    python3-rosdep=0.21.0-1 \
    python3-rosinstall=0.7.8-4 \
    python3-rosinstall-generator=0.1.22-1 \
    python3-wstool=0.1.18-2 \
    ros-noetic-robot=1.5.0-1focal.20220221.113143 \
    ros-noetic-ros-base=1.5.0-1focal.20220221.105927 \
    \
    # Jackal Clearpath Packages
    python-ds4drv=0.6.9-focal \
    ros-noetic-jackal-base=0.7.2-1focal.20220217.120629 \
    ros-noetic-jackal-bringup=0.7.2-1focal.20220217.122121 \
    \
    && rm -rf /var/lib/apt/lists/*


# ====================================================================
# | Root Filesystem Configuration                                    |
# ====================================================================

# Merge system folder config files into root filesystem
COPY ./root-filesystem/. /

# Add configuration at insertion point in global setup.bash
RUN sed -i '/^######$/i ROS_DISTRO='"${ROS_DISTRO}" /etc/ros/setup.bash && \
    # Initialize system-wide rosdep
    rosdep init

# Install default robot_upstart systemd jobs
RUN source /etc/ros/setup.bash && \
    /etc/ros/install_ros_bringup.py


# ====================================================================
# | User Configuration                                               |
# ====================================================================

# Set defaults for username and password
ARG JACKAL_USER="administrator"
ARG JACKAL_PASSWORD="clearpath"

# Add $JACKAL_USER user with group memberships and hashed password
RUN useradd -mUG "sudo" "${JACKAL_USER}" && \
    echo "${JACKAL_USER}:${JACKAL_PASSWORD}" \
    | chpasswd

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
