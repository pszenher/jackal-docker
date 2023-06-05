ARG ROS_DISTRO
FROM pszenher/jackal:${ROS_DISTRO}

# ====================================================================
# | Source Packages Compilation/Installation                         |
# ====================================================================

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN ${APT_UPDATE} && ${APT_INSTALL} \
    # Linux kernel/init packages
    initramfs-tools=0.136ubuntu6.7 \
    linux-image-generic=5.4.0.150.148 \
    systemd-sysv=245.4-4ubuntu3.21 \
    dhcpcd5=7.1.0-2build1 \
    && ${APT_PURGE_CACHE}

RUN ${APT_UPDATE} && ${APT_INSTALL} \
    # Sysadmin tool packages
    htop=2.2.0-2build1 \
    less=551-1ubuntu0.1 \
    openssh-server=1:8.2p1-4ubuntu0.7 \
    screen=4.8.0-1ubuntu0.1 \
    vim=2:8.1.2269-1ubuntu5.14 \
    && ${APT_PURGE_CACHE}

# Switch to $JACKAL_USER user (inherited from parent image)
USER "${JACKAL_USER}"
