FROM ubuntu:focal-20230412
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ focal main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ focal-security main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ focal-updates main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ focal-backports main universe multiverse restricted"
# apt-pin: repo="deb https://mirrors.osuosl.org/pub/ros/packages.ros.org/ros/ubuntu focal main"
# apt-pin: repo="deb https://packages.clearpathrobotics.com/stable/ubuntu focal main"


# ====================================================================
# Boilerplate Configuration
# ====================================================================

# Declare ROS distribution environment variable
ENV ROS_DISTRO=noetic

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Declare misc helper constants
ARG GITHUB="https://github.com"
ARG JACKAL_WS="/jackal_ws"

ARG GIT_CLONE="git clone --quiet --depth=1 --single-branch --config advice.detachedHead=false"
ARG APT_UPDATE="apt-get -qq update"
ARG APT_INSTALL="apt-get -qq install --no-install-recommends -y"
ARG APT_CACHE_PURGE="rm -rf /var/lib/apt/lists/*"

# Declare ROS package repository URLs
ARG ROS_URL="https://mirrors.osuosl.org/pub/ros/packages.ros.org"
ARG CLEARPATH_URL="https://packages.clearpathrobotics.com"

# Supress forced interaction during package installation
ARG DEBIAN_FRONTEND="noninteractive"
# Suppress debconf warnings (does not affect fatal errors)
ARG DEBCONF_NOWARNINGS="yes"
# Prevent install of recommended or suggested packages (needed for rosdep)
RUN echo 'APT::Install-Recommends "0";' >> /etc/apt/apt.conf.d/99-no-recommend && \
    echo 'APT::Install-Suggests   "0";' >> /etc/apt/apt.conf.d/99-no-recommend

# Configure locales
ENV LANG="C.UTF-8"
ENV LC_ALL="C.UTF-8"

# Configure timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime


# ====================================================================
# | Package Management and Configuration                             |
# ====================================================================

# Install boilerplate utilities
RUN ${APT_UPDATE} && ${APT_INSTALL} \
    \
    # System packages
    build-essential=12.8ubuntu1.1 \
    ca-certificates=20230311ubuntu0.20.04.1 \
    curl=7.68.0-1ubuntu2.18 \
    dhcpcd5=7.1.0-2build1 \
    gnupg=2.2.19-3ubuntu2.2 \
    python-is-python3=3.8.2-4 \
    software-properties-common=0.99.9.11 \
    \
    # Package management packages
    apt-transport-https=2.0.9 \
    python3-pip=20.0.2-5ubuntu1.8 \
    \
    # Linux kernel/init packages
    initramfs-tools=0.136ubuntu6.7 \
    linux-image-generic=5.4.0.150.148 \
    systemd-sysv=245.4-4ubuntu3.21 \
    \
    # Sysadmin tool packages
    git=1:2.25.1-1ubuntu3.11 \
    htop=2.2.0-2build1 \
    less=551-1ubuntu0.1 \
    openssh-server=1:8.2p1-4ubuntu0.7 \
    screen=4.8.0-1ubuntu0.1 \
    vim=2:8.1.2269-1ubuntu5.14 \
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
    python3-catkin-tools=0.9.2-1 \
    python3-osrf-pycommon=2.0.2-1 \
    python3-rosdep=0.22.2-1 \
    python3-rosinstall=0.7.8-4 \
    python3-rosinstall-generator=0.1.23-1 \
    python3-wstool=0.1.18-2 \
    ros-noetic-robot=1.5.0-1focal.20230306.094205 \
    ros-noetic-ros-base=1.5.0-1focal.20230216.010618 \
    \
    # Jackal Clearpath Packages
    python-ds4drv=0.6.10-focal \
    ros-noetic-jackal-base=0.7.7-1focal.20230107.015635 \
    ros-noetic-jackal-bringup=0.7.7-1focal.20230107.015950 \
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

# Build home directory catkin workspace
# hadolint ignore=SC1091
RUN source /etc/ros/setup.bash && \
    mkdir -p "/home/${JACKAL_USER}/catkin_ws/src" && \
    catkin build --workspace "/home/${JACKAL_USER}/catkin_ws"

# Set working directory to user home dir
WORKDIR "/home/${JACKAL_USER}"

# Set default command to bash
CMD ["/bin/bash"]
