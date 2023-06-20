ARG UBUNTU_DISTRO="focal"
ARG ROS_VERSION="ros"
ARG ROS_DISTRO="noetic"
FROM pszenher/jackal-base:${UBUNTU_DISTRO}

# Apply ROS configuration as root user
USER "root"

# Install deb packages from newly installed repos
RUN apt-get update && apt-get install --no-install-recommends -y \
    \
    python3-pip \
    git \
    \
    && ${APT_CACHE_PURGE}

RUN pip install \
    rosinstall_generator \
    vcstool \
    rosdep \
    catkin_tools

RUN ${GIT_CLONE} "https://github.com/ros/rosdistro" \
    "/tmp/rosdistro" \
    --branch "noetic/2023-06-15"

COPY ./focal/rosdep-sources.list /etc/ros/rosdep/sources.list.d/20-default.list
COPY ./focal/rosdistro-index-v4.yaml /tmp/rosdistro/index-custom.yaml

RUN rm /tmp/rosdistro/index-v4.yaml

RUN rosdistro_build_cache \
    --preclean \
    --ignore-local \
    --debug \
    "file:///tmp/rosdistro/index-custom.yaml" \
    "noetic"
RUN cp noetic-cache.yaml.gz /tmp

RUN ${GIT_CLONE} "https://github.com/clearpathrobotics/public-rosdistro" \
    "/tmp/rosdistro-clearpath" \
    --branch "master"

# Generate from-source ros installation
RUN mkdir -p /tmp/ros_ws/src

RUN ROSDISTRO_INDEX_URL="file:///tmp/rosdistro/index-custom.yaml" \
    rosinstall_generator robot --rosdistro noetic --deps --tar \
    | vcs import /tmp/ros_ws/src

# Initial run of rosdep for base ros deps, speed up devel loop...
RUN ROSDISTRO_INDEX_URL="file:///tmp/rosdistro/index-custom.yaml" \
    rosdep update --rosdistro noetic && \
    apt-get update && \
    rosdep install \
    --from-paths /tmp/ros_ws/src \
    --ignore-src \
    --default-yes \
    --simulate \
    --rosdistro noetic \
    --os ubuntu:focal \
    --skip-keys jackal_tests \
    --skip-keys jackal_firmware \
    --skip-keys python3-catkin-pkg-modules \
    --skip-keys python3-rosdep-modules \
    --skip-keys lms1xx \
    | sort \
    | sed -n 's/^ *apt-get install\( -y\)\? \([A-z0-9-]*\)$/\2/p' \
    | xargs apt-get install -y --no-install-recommends

RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    jq \
    && \
    pip install yq

ARG ROS_VERSION="ros"
ARG ROS_DISTRO="noetic"

RUN catkin config --init --workspace "/tmp/ros_ws" --install-space "/opt/${ROS_VERSION}/${ROS_DISTRO}" --install && \
    catkin build \
    --workspace "/tmp/ros_ws" \
    -DCMAKE_BUILD_TYPE=Release

RUN yq -y \
    'setpath( \
    ["repositories"]; \
    .repositories | with_entries(select(  .. | .url? | strings | contains("gitlab.clearpathrobotics.com") | not )))' \
    /tmp/rosdistro-clearpath/noetic/distribution.yaml \
    > /tmp/rosdistro-clearpath/noetic/distribution-intermediate.yaml

# Merge upstream and clearpath rosdistro files
RUN yq -s '.[0] * .[1]' \
    /tmp/rosdistro/noetic/distribution.yaml \
    /tmp/rosdistro-clearpath/noetic/distribution-intermediate.yaml \
    > /tmp/rosdistro-clearpath/noetic/distribution-full.yaml

COPY ./focal/clearpath-index-v4.yaml /tmp/rosdistro-clearpath/index-custom.yaml

RUN rosdistro_build_cache \
    --debug \
    "file:///tmp/rosdistro-clearpath/index-custom.yaml" \
    "noetic"

# Copy full cache (including clearpath) to main cache paths from index.yaml
# TODO:  should this be separate, rather than overriding the base cache?
RUN cp noetic-cache.yaml.gz /tmp/noetic-cache.yaml.gz

RUN ROSDISTRO_INDEX_URL="file:///tmp/rosdistro-clearpath/index-custom.yaml" \
    rosinstall_generator jackal_robot --rosdistro noetic --deps --tar \
    # HACK: monkey-patch lms1xx release yaml, due to clearpath changing the name of the repo, breaking vcs...
    | sed 's/lms1xx-release-release-noetic-lms1xx-0.3.0-2/LMS1xx-release-release-noetic-lms1xx-0.3.0-2/' \
    | vcs import /tmp/ros_ws/src

RUN ROSDISTRO_INDEX_URL="file:///tmp/rosdistro/index-custom.yaml" \
    rosdep update --rosdistro noetic && \
    apt-get update && \
    rosdep install \
    --from-paths /tmp/ros_ws/src \
    --ignore-src \
    --default-yes \
    --simulate \
    --rosdistro noetic \
    --os ubuntu:focal \
    --skip-keys jackal_tests \
    --skip-keys jackal_firmware \
    --skip-keys python3-catkin-pkg-modules \
    --skip-keys python3-rosdep-modules \
    | sort \
    | sed -n 's/^ *apt-get install\( -y\)\? \([A-z0-9-]*\)$/\2/p' \
    | xargs apt-get install -y --no-install-recommends

RUN apt update && apt install -y --no-install-recommends \
    # Needed by velodyne_driver
    libpcap-dev \
    # Needed by sick_tim
    libusb-1.0-0-dev

RUN catkin build \
    --workspace "/tmp/ros_ws" \
    -DCMAKE_BUILD_TYPE=Release


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
    cp "$path" "$d"' \;

# Add configuration to global setup.bash
RUN sed -i '/^######$/i export ROS_DISTRO='"${ROS_DISTRO}"  /etc/ros/setup.bash

RUN apt-get update && apt-get install \
    python-is-python3 \
    sudo \
    # TODO: testing pkg, remove
    ncdu

# Install default robot_upstart systemd jobs
RUN source /etc/ros/setup.bash && \
    /etc/ros/install_ros_bringup.py

# FIXME: $JACKAL_USER is not set here?
# Return to $JACKAL_USER user
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
# Use entrypoint to configure ros env when running in Docker container
ENTRYPOINT ["/etc/ros/ros_entrypoint.sh"]
