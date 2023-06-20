ARG DEB_SUITE="focal"
FROM pszenher/ubuntu-bootstrap:${DEB_SUITE}

# Declare ROS distribution environment variable
ARG DEB_SUITE="focal"
ARG ROS_DISTRO="noetic"
ARG ROS_TYPE="ros"

# Declare ROS package repository URLs
ARG ROS_URL="https://mirrors.osuosl.org/pub/ros/packages.ros.org"
ARG CLEARPATH_URL="https://packages.clearpathrobotics.com"

# Instal GPG for adding apt packaging keys
RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    gnupg \
    \
    && rm -rf /var/lib/apt/lists/*

# Add ROS and Clearpath deb package repos
COPY ./keys/ros.asc       /tmp/ros.asc
COPY ./keys/clearpath.asc /tmp/clearpath.asc
RUN apt-key add /tmp/ros.asc       && \
    apt-key add /tmp/clearpath.asc && \
    echo "deb" "${ROS_URL}/${ROS_TYPE}/ubuntu/" "${DEB_SUITE}" "main" \
    > /etc/apt/sources.list.d/ros-latest.list && \
    echo "deb" "${CLEARPATH_URL}/stable/ubuntu/"   "${DEB_SUITE}" "main" \
    > /etc/apt/sources.list.d/clearpath-latest.list

# Install deb packages from newly installed repos
RUN DEBIAN_FRONTEND="noninteractive" \
    DEBCONF_NOWARNINGS="yes" \
    apt-get update     && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    \
    python3-rosdep=0.22.2-1 \
    git \
    python3-pip \
    jq \
    \
    && rm -rf "/var/lib/apt/lists/*"

# Install yq for distribution.yaml file merging
RUN pip install yq

# Clone upstream rosdistro snapshot
RUN git clone \
    --quiet \
    --depth=1 \
    --single-branch \
    --config advice.detachedHead=false \
    --branch "noetic/2023-06-15" \
    "https://github.com/ros/rosdistro" \
    "/tmp/rosdistro"

# Clone upstream rosdistro snapshot
RUN git clone \
    --quiet \
    --depth=1 \
    --single-branch \
    --config advice.detachedHead=false \
    --branch "master" \
    "https://github.com/clearpathrobotics/public-rosdistro" \
    "/tmp/rosdistro-clearpath"

# Copy modified index.yaml, and set as default for subsequent rosdep commands
ENV ROSDISTRO_INDEX_URL="file:///etc/ros/rosdistro/index-v4.yaml"

# Copy clearpath noetic distribution.yaml, and merge with upstream
COPY ./focal/rosdistro_merge.sh /usr/bin/rosdistro_merge
RUN rosdistro_merge -o /etc/ros/rosdistro /tmp/rosdistro-clearpath /tmp/rosdistro

# Initialize rosdep sources.list(s) from host copy 
COPY ./focal/sources.list.d /etc/ros/rosdep/sources.list.d

# Clone jackal codebases
RUN git clone \
    --quiet \
    --depth=1 \
    --single-branch \
    --config advice.detachedHead=false \
    --branch "noetic-devel" \
    "https://github.com/jackal/jackal" \
    "/tmp/jackal/jackal_core" \
    && \
    # Instantly purge jackal nav and tutorial packages
    rm -rf "/tmp/jackal/jackal_core/jackal_navigation" && \
    rm -rf "/tmp/jackal/jackal_core/jackal_tutorials"

# TODO: downsample jackal_description meshes (esp. bridge-plate.stl),
#       if not delete entirely...

# FIXME: ros-noetic-rosconsole depends on boost-regex-dev rather than
#        boost-regex, due to:
#        https://github.com/ros/rosconsole/pull/38#issue-564202749
#        this results in excessive Boost dep inclusion
#
# NOTE: upon review, this is the case all over the ROS package suite,
#       including roscpp (why are there 3 boost-*-dev ppackages set as
#       run-depends?

RUN git clone \
    --quiet \
    --depth=1 \
    --single-branch \
    --config advice.detachedHead=false \
    --branch "noetic-devel" \
    "https://github.com/jackal/jackal_robot" \
    "/tmp/jackal/jackal_robot"

# Install jackal packages from custom package.xml metapackage
RUN rosdep update --rosdistro "${ROS_DISTRO}"
COPY ./focal/jackal-package.xml /tmp/jackal/jackal_rfal/package.xml

# Monkey-patch package.xml files to remove uneeded packages
COPY ./focal/strip_package_xml.sh /usr/bin/strip_package_xml
RUN find /tmp/jackal -type f -name "package.xml" -exec strip_package_xml {} \;    

# Add held-package preferences to block install of upstream ros-noetic-jackal* packages
COPY ./focal/90-jackal.pref /etc/apt/preferences.d/90-jackal.pref

# Install Jackal ROS dependencies
RUN apt-get update && \
    rosdep install \
    --from-paths /tmp/jackal \
    --default-yes \
    --ignore-src \
    --simulate \
    | sort \
    | sed -n 's/^ *apt-get install\( -y\)\? \([A-z0-9-]*\)$/\2/p' \
    | tee "/dev/stderr" \
    | xargs apt-get install -y --no-install-recommends \
    \
    && rm -rf "/var/lib/apt/lists/*"

# Install utilities for catkin build execution
RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    make \
    gcc-8 \
    g++-8 \
    \
    && rm -rf /var/lib/apt/lists/*

# Manually trigger update-alternatives for gcc and g++ (not done automatically?)
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 10 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 10

# FIXME: painfully blatant hack...
# Monkey-patch jackal_base to remove CMake dep on teleop_twist_joy
RUN sed -i 's/teleop_twist_joy//' "/tmp/jackal/jackal_robot/jackal_base/CMakeLists.txt"

# FIXME: move alongside package.xml copy op (or make a proper package...)
COPY ./focal/jackal-package.CMakeLists /tmp/jackal/jackal_rfal/CMakeLists.txt

# FIXME: should put these in */src originally, patching to limit dev recomp. time
RUN mkdir -p /tmp/jackal/src && \
    mv /tmp/jackal/{jackal_core,jackal_robot,jackal_rfal} /tmp/jackal/src

RUN source "/opt/${ROS_TYPE}/${ROS_DISTRO}/setup.bash" && \
    catkin config --init --workspace "/tmp/jackal" --install-space "/opt/${ROS_TYPE}/${ROS_DISTRO}" --install && \
    catkin build \
    --workspace "/tmp/jackal" \
    -DCMAKE_BUILD_TYPE=Release

# Patch jackal_description to allow missing unused sensor drivers, rebuild
COPY ./focal/jackal-description-accessories.urdf.xacro \
    /tmp/jackal/src/jackal_core/jackal_description/urdf/accessories.urdf.xacro
RUN source "/opt/${ROS_TYPE}/${ROS_DISTRO}/setup.bash" && \
    catkin build \
    --workspace "/tmp/jackal" \
    -DCMAKE_BUILD_TYPE=Release

# TODO: jackal.urdf.xacro cannot be processed without full sensor
#       stack (sick_tim, etc.), patch to remove extraneous sensors...

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

# Add configuration at insertion point in global setup.bash
RUN sed -i '/^######$/i export ROS_DISTRO='"${ROS_DISTRO}" /etc/ros/setup.bash

# Install Python for ros bringup script execution
RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    python-is-python3 \
    ncdu \
    hardlink \
    \
    && rm -rf /var/lib/apt/lists/*

# Install default robot_upstart systemd jobs
RUN source /etc/ros/setup.bash && \
    "/etc/ros/install_${ROS_TYPE}_bringup.py"


# ====================================================================
# | User Configuration                                               |
# ====================================================================

# FIXME: this should propagate from some global config...
# Set defaults for username and password
ARG JACKAL_USER="administrator"
ARG JACKAL_PASSWORD="clearpath"

# Add $JACKAL_USER user with group memberships and hashed password
RUN useradd -mUG "sudo" -s "/bin/bash" "${JACKAL_USER}" && \
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
    mkdir -p "/home/${JACKAL_USER}/catkin_ws/src"

# Set working directory to user home dir
WORKDIR "/home/${JACKAL_USER}"

# Use entrypoint to configure ros env when running in Docker container
ENTRYPOINT ["/etc/ros/ros_entrypoint.sh"]

# Re-set CMD, as its parent value is cleared by setting ENTRYPOINT
CMD        ["/usr/bin/env", "bash"]
