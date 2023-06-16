ARG FINAL_IMAGE=pszenher/jackal-ros-source:noetic
FROM ${FINAL_IMAGE}

# Return to root user (and stay there, as this image is only for
# whaleboot)
USER root

# ================================================================
# |  Install Packages required for sucessful disk image flash
# ================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    linux-virtual \
    initramfs-tools \
    systemd-sysv \
    \
    && rm -rf /var/lib/apt/lists/*


# ================================================================
# |  Slash and Burn (disk-size) Agriculture
# ================================================================

# Purge unnecessary development packages
RUN apt purge -y \
    \
    git \
    \
    && apt autoremove -y

# Purge excessively large files (in order of increasing risk of breakage)
RUN rm -r \
    \
    # INFO: no problem, meant to be deleted
    # TODO: copy relevant build stuff from tmp to main fs
    /tmp/* \
    /var/lib/apt/lists/* \
    \
    # WARN: may be unsafe, check rammifications of purging apt cache
    /var/cache/* \
    \
    # WARN: dangerous, probably a better way... downsample mesh programmatically?
    /opt/ros/noetic/share/realsense2_description/meshes/* \
    /opt/ros/noetic/lib/libSpinnaker.so* \
    /opt/ros/noetic/lib/libflycapture.so* \
    \
    # WARN++: VERY dangerous, almost certainly a better way... dpkg-divert?
    /usr/lib/x86_64-linux-gnu/libLLVM-12.so.1 \
    /usr/lib/x86_64-linux-gnu/dri \
    /usr/share/doc/* \
    /usr/share/icons/* \
    /usr/share/man/* \
    /usr/share/X11/*

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
