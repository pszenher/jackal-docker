FROM ubuntu:xenial-20210416
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ xenial main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ xenial-security main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ xenial-updates main universe multiverse restricted"
# apt-pin: repo="deb http://archive.ubuntu.com/ubuntu/ xenial-backports main universe multiverse restricted"
# apt-pin: repo="deb https://mirrors.osuosl.org/pub/ros/packages.ros.org/ros/ubuntu xenial main"
# apt-pin: repo="deb https://packages.clearpathrobotics.com/stable/ubuntu xenial main"

ENV ROS_DISTRO=kinetic
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
    build-essential=12.1ubuntu2 \
    ca-certificates=20210119~16.04.1 \
    curl=7.47.0-1ubuntu2.19 \
    dhcpcd5=6.10.1-1 \
    gnupg2=2.1.11-6ubuntu2.1 \
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
    git=1:2.7.4-0ubuntu1.10 \
    htop=2.0.1-1ubuntu1 \
    less=481-2.1ubuntu0.2 \
    openssh-server=1:7.2p2-4ubuntu2.10 \
    screen=4.3.1-2ubuntu0.1 \
    vim=2:7.4.1689-3ubuntu1.5 \
    wicd-curses=1.7.4+tb2-1 \
    \
    && rm -rf /var/lib/apt/lists/*

# Add ROS and Clearpath deb package repos
RUN echo "deb ${ROS_URL}/ros/ubuntu/ xenial main" > /etc/apt/sources.list.d/ros-latest.list &&\
    curl -sSL "${ROS_URL}/ros.key" | apt-key add - \
    && \
    echo "deb ${CLEARPATH_URL}/stable/ubuntu xenial main" > /etc/apt/sources.list.d/clearpath-latest.list &&\
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
    ros-kinetic-robot=1.3.2-0xenial-20210503-155848-0800 \
    ros-kinetic-ros-base=1.3.2-0xenial-20210503-151705-0800 \
    \
    # Jackal Clearpath Packages
    python-ds4drv=0.5.2xenial \
    ros-kinetic-jackal-robot=0.5.1-1xenial-20210608-235325-0500 \
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
# Set username as ENV for inheritance into child image builds
ENV JACKAL_USER="administrator"
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
