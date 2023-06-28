ARG FINAL_IMAGE=pszenher/jackal-ros-bin:noetic
FROM ${FINAL_IMAGE}

# Return to root user (and stay there, as this image is only for
# whaleboot)
USER root

# Disable 30s resume disk search during initramfs boot phase
RUN mkdir -p "/etc/initramfs-tools/conf.d" && \
    echo "RESUME=none" > /etc/initramfs-tools/conf.d/no-resume.conf

# ================================================================
# |  Install Packages required for sucessful disk image flash
# ================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    # Ubuntu Linux Kernel, minimal modules
    linux-image-generic \
    # Build initrd.img
    initramfs-tools \
    # SystemD init system
    systemd-sysv \
    # Needed for user systemctl invocation
    dbus-user-session \
    # ip command suite
    iproute2 \
    # wifi authentication utils
    wpasupplicant \
    # sshd
    openssh-server \
    # pci & usb utils
    pciutils \
    usbutils \
    # bluetooth utils
    bluez \
    \
    && rm -rf /var/lib/apt/lists/*

RUN systemctl enable systemd-networkd && \
    # Enable wpa_supplicant auth on wifi0 dev
    systemctl enable wpa_supplicant@wifi0
    # && \
    # # Enable systemd-resolved DNS handling
    # ln -sf /run/systemd/resolve/stub-resolv.conf /etc/resolv.conf

# ================================================================
# |  Slash and Burn (disk-size) Agriculture
# ================================================================

# Purge unnecessary development packages
RUN apt-get purge -y \
    \
    git \
    \
    && apt-get autoremove -y

# Purge excessively large files (in order of increasing risk of breakage)
RUN rm -rf \
    \
    # TODO: copy relevant build stuff from tmp to main fs
    # TODO: purge /etc/ros/rosdistro/!(${ROS_DISTRO}|rosdep)
    /tmp/* \
    /var/lib/apt/lists/* \
    /var/cache/* \
    /var/log/* \
    /usr/share/icons/* \
    /usr/share/man/* \
    /usr/share/cmake-*/Help/*

# Deduplicate copyright files in documentation system dir
RUN hardlink -t /usr/share/doc

# TODO: actually define these semantics...
# Add whaleboot manifest to final image
COPY focal/whaleboot-manifest.yaml /whaleboot-manifest.yaml

# # NOTE:  temporary testing of whaleboot config...
# # FIXME: remove
# RUN  curl \
#     "https://dl-cdn.alpinelinux.org/alpine/v3.18/releases/x86_64/alpine-virt-3.18.2-x86_64.iso" \
#     > /tmp/alpine.iso

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     qemu-system-x86

# RUN  qemu-system-x86_64 \
#     -boot d \
#     -cdrom alpine.iso \
#     -hda test.img \
#     -drive file=image.iso,media=cdrom \
#     -machine q35 \
#     -m 8G \
#     -smp 8 \
#     -display none \
#     -serial mon:stdio \
#     -drive file=/tmp/qemushare/rootfs.tar.gz,format=raw
