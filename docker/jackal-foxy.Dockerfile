FROM ubuntu:focal-20230412


# ====================================================================
# Boilerplate Configuration
# ====================================================================

# Configure user and build shell shell
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Declare ROS distribution environment variable
ENV ROS_DISTRO=foxy

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
    # Software management packages
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
    && ${APT_CACHE_PURGE}

# Add ROS and Clearpath deb package repos
RUN echo "deb ${ROS_URL}/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list &&\
    curl -sSL "${ROS_URL}/ros.key" | apt-key add -

# Install deb packages from newly installed repos
RUN ${APT_UPDATE} && ${APT_INSTALL} \
    \
    python3-rosdep=0.22.2-1 \
    python3-colcon-common-extensions \
    \
    && ${APT_CACHE_PURGE}

# Install jackal-robot packages from source (not yet packaged for Foxy)
RUN mkdir -p "${JACKAL_WS}/src" \
    && \
    ${GIT_CLONE} "${GITHUB}/jackal/jackal_robot.git"       "${JACKAL_WS}/src/jackal_robot" \
    --branch "1.0.1" \
    && \
    ${GIT_CLONE} "${GITHUB}/micro-ROS/micro-ros-agent.git" "${JACKAL_WS}/src/micro_ros_agent" \
    --branch "1.5.4" \
    && \
    ${GIT_CLONE} "${GITHUB}/jackal/jackal.git"             "${JACKAL_WS}/src/jackal" \
    --branch "1.0.4" \
    && \
    # Purge jackal navigation subpackage (drags in 500MB of rosdeps)
    rm -rf "${JACKAL_WS}/src/jackal/jackal_navigation"

# Install ROS dependencies for jackal source packages
RUN rosdep  init   && \
    rosdep  update && \
    apt-get update && \
    rosdep install -iy --from-paths "${JACKAL_WS}/src" && \
    rm -rf "${HOME}/.ros"

# Build chained colcon workspace for jackal source packages
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build \
    --merge-install \
    --symlink-install \
    --base-paths   "${JACKAL_WS}/src" \
    --build-base   "${JACKAL_WS}/build" \
    --install-base "${JACKAL_WS}/install"


# ====================================================================
# | Root Filesystem Configuration                                    |
# ====================================================================

# Merge system folder config files into root filesystem
COPY ./root-filesystem/. /

# Set system networking config
RUN echo "jackal-${ROS_DISTRO}"                > "/etc/hostname" && \
    echo "127.0.0.1	localhost"             > "/etc/hosts"    && \
    echo "127.0.1.1	jackal-${ROS_DISTRO}" >> "/etc/hosts"

# Add configuration to global setup.bash
RUN sed -i '/^######$/i ROS_DISTRO='"${ROS_DISTRO}"  /etc/ros/setup.bash && \
    echo "source ${JACKAL_WS}/install/setup.bash" >> /etc/ros/setup.bash

# Install default robot_upstart systemd jobs
RUN source "/etc/ros/setup.bash" && /etc/ros/install_ros2_bringup.py


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

# # Symlink kernel into fs root directory
# RUN ln -s /boot/vmlinuz /vmlinuz && \
#     ln -s /boot/initrd.img /initrd.img

# Install Ubuntu live boot enablement package(s)
RUN ${APT_UPDATE} && ${APT_INSTALL} \
    \
    live-boot \
    live-boot-initramfs-tools \
    extlinux \
    squashfs-tools \
    \
    && ${APT_CACHE_PURGE}

# TODO: include bluez, bluez-tools, python-ds4drv in this?
# (from jackal.sh in new install img)

# Switch to $JACKAL_USER user
USER "${JACKAL_USER}"

# Set working directory to user home dir
WORKDIR "/home/${JACKAL_USER}"

# Set default command to bash
CMD ["/bin/bash"]

# FIXME: probably need this kernel boot arg to prevent the MCU from
#        killing the power supply due to usb interaction timeout w/
#        new linux kernel usb enum scheme and microstrain imu
# 
# usbcore.old_scheme_first=Y
