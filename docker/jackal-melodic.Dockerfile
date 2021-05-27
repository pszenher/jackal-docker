FROM ubuntu:bionic-20210416
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ bionic main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ bionic-security main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ bionic-updates main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ bionic-backports main universe multiverse restricted"
# apt-pin: repo="deb https://mirrors.osuosl.org/pub/ros/packages.ros.org/ros/ubuntu bionic main"
# apt-pin: repo="deb https://packages.clearpathrobotics.com/stable/ubuntu bionic main"

ENV ROS_DISTRO=melodic

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
    build-essential=12.4ubuntu1 \
    ca-certificates=20210119~18.04.1 \
    curl=7.58.0-2ubuntu3.13 \
    dhcpcd5=6.11.5-0ubuntu1 \
    gnupg=2.2.4-1ubuntu1.4 \
    software-properties-common=0.96.24.32.14 \
    \
    # Package management packages
    apt-transport-https=1.6.13 \
    python-pip=9.0.1-2.3~ubuntu1.18.04.4 \
    python3-pip=9.0.1-2.3~ubuntu1.18.04.4 \
    \
    # Linux kernel/init packages
    initramfs-tools=0.130ubuntu3.12 \
    linux-image-generic=4.15.0.143.130 \
    systemd-sysv=237-3ubuntu10.47 \
    \
    # Sysadmin tool packages
    git=* \
    htop=2.1.0-3 \
    less=487-0.1 \
    openssh-server=1:7.6p1-4ubuntu0.3 \
    screen=4.6.2-1ubuntu1.1 \
    vim=2:8.0.1453-1ubuntu1.4 \
    wicd-curses=1.7.4+tb2-5 \
    \
    && rm -rf /var/lib/apt/lists/*

# Add ROS and Clearpath deb package repos
RUN echo "deb ${ROS_URL}/ros/ubuntu/ bionic main" > /etc/apt/sources.list.d/ros-latest.list &&\
    curl -sSL "${ROS_URL}/ros.key" | apt-key add - \
    && \
    echo "deb ${CLEARPATH_URL}/stable/ubuntu bionic main" > /etc/apt/sources.list.d/clearpath-latest.list &&\
    curl -sSL "${CLEARPATH_URL}/public.key" | apt-key add -

# Install deb packages from newly installed repos
RUN apt-get update && apt-get install --no-install-recommends -y \
    \
    # Jackal ROS Packages
    python-catkin-tools=0.6.1-1 \
    python-rosdep=0.20.1-1 \
    python-rosinstall=0.7.8-1 \
    python-rosinstall-generator=0.1.22-1 \
    python-wstool=0.1.17-1 \
    ros-melodic-robot=1.4.1-0bionic.20210505.053322 \
    ros-melodic-ros-base=1.4.1-0bionic.20210505.053238 \
    \
    # Jackal Clearpath Packages
    python-ds4drv=0.5.8-bionic \
    ros-melodic-jackal-robot=0.6.2-1bionic.20210429.202740 \
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
