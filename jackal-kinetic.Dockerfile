FROM ubuntu:xenial-20210416

ENV ROS_DISTRO=kinetic

ARG ROS_URL="https://mirrors.osuosl.org/pub/ros/packages.ros.org"
ARG CLEARPATH_URL="https://packages.clearpathrobotics.com"

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install boilerplate utilities
RUN DEBIAN_FRONTEND=noninteractive \
    apt-get update && apt-get install --no-install-recommends -y \
    \
    # System packages
    build-essential \
    ca-certificates=* \
    curl=7.47.* \
    dhcpcd5=6.10.* \
    software-properties-common=0.96.20.* \
    \
    # Package management packages
    apt-transport-https=1.2.* \
    python-pip \
    python3-pip \
    \
    # Linux kernel/init packages
    initramfs-tools=0.122* \
    linux-image-generic=4.4.* \
    systemd-sysv=229-4* \
    \
    # Sysadmin tool packages
    htop=2.0.* \
    less \
    openssh-server=1:7.2* \
    screen \
    vim \
    wicd-curses \
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
    # ROS Packages
    python-rosdep=0.20.* \
    python-rosinstall=0.7.* \
    python-rosinstall-generator \
    python-wstool \
    ros-kinetic-robot=1.3.2* \
    ros-kinetic-ros-base=1.3.2* \
    \
    # Clearpath Packages
    python-ds4drv=0.5.2xenial \
    ros-kinetic-jackal-robot=0.5.* \
    \
    && rm -rf /var/lib/apt/lists/*

# Initialize system-wide rosdep
RUN rosdep init

# Merge system folder config files into root filesystem
COPY ./root-filesystem/. /

# Install jackal bringup systemd scripts
RUN source /etc/ros/setup.bash && \
    /etc/ros/install-ros-bringup.py

# Add rfal user with group memberships and SHA256 hashed password
RUN useradd -mUG sudo rfal && \
    echo 'rfal:$6$lTlXs59jEbj2Je$zZ/eQ5S142JWYrglIpJq8NtDo8uKkIu6WkTcucVWy.7uRk2O/1DOQMMWgLi6/Lkvm1OlkPnKpRFrVXGRM5rNG0' \
    | chpasswd -e

# Switch to rfal user
USER rfal

# Update rosdep as rfal user
RUN rosdep update

CMD ["/bin/bash"]
