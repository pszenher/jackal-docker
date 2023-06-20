ARG UBUNTU_DISTRO="focal"
ARG ROS_VERSION="ros"
ARG ROS_DISTRO="noetic"
FROM pszenher/jackal-ros-source:${ROS_DISTRO}

USER root

RUN apt-get update && apt-get install -y \
    libgazebo9-dev

RUN rosinstall_generator jackal_simulator --rosdistro noetic --deps --tar \
    | sed 's/lms1xx-release-release/LMS1xx-release-release/' \
    | vcs import --skip-existing /tmp/ros_ws/src

RUN rosdep install -isy \
    --from-paths /tmp/ros_ws/src \
    --skip-keys jackal_tests \
    --skip-keys jackal_firmware

RUN catkin build \
    --workspace /tmp/ros_ws \
    -DCMAKE_BUILD_TYPE=Release

RUN apt-get install -y \
    gazebo9 \
    nodejs \
    npm \
    # gzweb deps:
    libjansson-dev \
    libboost-dev \
    imagemagick \
    libtinyxml-dev \
    mercurial \
    cmake \
    build-essential

RUN ${GIT_CLONE} "https://github.com/osrf/gzweb.git" \
    "/tmp/gzweb" \
    --branch "gzweb_1.4.1"

WORKDIR /tmp/gzweb

# Build gzweb with models
RUN npm run deploy --- -m

COPY /focal/gazebo_test.launch /tmp
COPY /focal/gazebo_spawn.launch /tmp
COPY /focal/gazebo_config.env /tmp

# # Build/install foxglove_bridge
# RUN rosinstall_generator foxglove_bridge --rosdistro noetic --deps --tar \
#     | sed 's/lms1xx-release-release/LMS1xx-release-release/' \
#     | vcs import --skip-existing /tmp/ros_ws/src

# RUN rosdep install -isy \
#     --from-paths /tmp/ros_ws/src \
#     --skip-keys jackal_tests \
#     --skip-keys jackal_firmware

# RUN catkin build \
#     --workspace /tmp/ros_ws \
#     -DCMAKE_BUILD_TYPE=Release
