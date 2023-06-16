# ARG UBUNTU_DIST="focal"
# ARG UBUNTU_IMAGE="${UBUNTU_DIST}-20230412"
# FROM ubuntu:${UBUNTU_IMAGE}
FROM pszenher/ubuntu-bootstrap:focal

# ====================================================================
# Boilerplate Configuration
# ====================================================================

# Configure user and build shell shell
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Declare misc helper constants
ENV GITHUB="https://github.com"
ENV JACKAL_WS="/jackal_ws"

ENV GIT_CLONE="git clone --quiet --depth=1 --single-branch --config advice.detachedHead=false"
ENV APT_UPDATE="apt-get -q update"
ENV APT_INSTALL="apt-get -q install --no-install-recommends -y"
ENV APT_CACHE_PURGE="rm -rf /var/lib/apt/lists/*"

# Declare ROS package repository URLs
ENV ROS_URL="https://mirrors.osuosl.org/pub/ros/packages.ros.org"
ENV CLEARPATH_URL="https://packages.clearpathrobotics.com"

# Supress forced interaction during package installation
ENV DEBIAN_FRONTEND="noninteractive"
# Suppress debconf warnings (does not affect fatal errors)
ENV DEBCONF_NOWARNINGS="yes"


# ====================================================================
# | Boilerplate Disk Configuration
# ====================================================================

# Prevent install of recommended or suggested packages (needed for rosdep)
RUN echo 'APT::Install-Recommends "0";' >> /etc/apt/apt.conf.d/99-no-recommend && \
    echo 'APT::Install-Suggests   "0";' >> /etc/apt/apt.conf.d/99-no-recommend

# Configure locales
ENV LANG="C.UTF-8"
ENV LC_ALL="C.UTF-8"

# Configure timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

# Upgrade bootstrapped packages and get tls certs
RUN apt-get update     && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    \
    ca-certificates \
    \
    && rm -rf /var/lib/apt/lists/*


# ====================================================================
# | User Configuration                                               |
# ====================================================================

# Set defaults for username and password
ENV JACKAL_USER="administrator"
ENV JACKAL_PASSWORD="clearpath"

# Add $JACKAL_USER user with group memberships and hashed password
RUN useradd -mUG "sudo" -s "/bin/bash" "${JACKAL_USER}" && \
    echo "${JACKAL_USER}:${JACKAL_PASSWORD}" \
    | chpasswd

# Copy home directory contents into $JACKAL_USER home dir and set ownership
COPY ./home-directory/. "/home/${JACKAL_USER}"
RUN chown -R "${JACKAL_USER}:${JACKAL_USER}" "/home/${JACKAL_USER}"

# Switch to $JACKAL_USER user
USER "${JACKAL_USER}"

# Set working directory to user home dir
WORKDIR "/home/${JACKAL_USER}"

# Set default command to bash
CMD ["/bin/bash"]
