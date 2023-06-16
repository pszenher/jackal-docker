FROM pszenher/jackal:base

ARG ROS_VERSION="ros"
ARG ROS_DISTRO="noetic"

# Apply ROS configuration as root user
USER "root"

COPY ./keys /tmp/keys

# Add ROS deb package repos
RUN ${APT_UPDATE} && \
    ${APT_INSTALL} gnupg=2.2.19-3ubuntu2.2 &&  \
    cat "/tmp/keys/ros.key" | apt-key add - && \
    apt purge -qy gnupg=2.2.19-3ubuntu2.2 && \
    apt autoremove -qy && \
    echo "deb ${ROS_URL}/${ROS_VERSION}/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list

# Install deb packages from newly installed repos
RUN ${APT_UPDATE} && ${APT_INSTALL} \
    \
    "ros-noetic-ros-core" \
    "python3-rosdep=0.22.2-1" \
    \
    && ${APT_CACHE_PURGE}


# ====================================================================
# | Root Filesystem Configuration                                    |
# ====================================================================

# Merge system folder config files into root filesystem
COPY ./root-filesystem /tmp/docker-root-overlay
RUN cd /tmp/docker-root-overlay && \
    find . \
    -type f \
    -exec bash -c \
    'path="{}"; \
    d=/$(dirname "$path"); \
    mkdir -p "$d" ; \
    cp "$path" "$d"' \; \
    && rm -rf /tmp/*

# Add configuration to global setup.bash
RUN sed -i '/^######$/i ROS_DISTRO='"${ROS_DISTRO}"  /etc/ros/setup.bash && \
    echo "source ${JACKAL_WS}/install/setup.bash" >> /etc/ros/setup.bash

# RUN ln -s \
#     /lib/systemd/system/${ROS_VERSION}.service \
#     /etc/systemd/system/multi-user.target.wants/${ROS_VERSION}.service

# Return to $JACKAL_USER user
USER "${JACKAL_USER}"
