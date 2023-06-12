FROM ubuntu:focal

ARG APT_UPDATE="apt-get -qq update"
ARG APT_INSTALL="apt-get -qq install --no-install-recommends -y"
ARG APT_CACHE_PURGE="rm -rf /var/lib/apt/lists/*"

ARG SOURCE_LIVE_SCRIPTS=". /usr/lib/live/build.sh"

ARG BUILD_DIR="/tmp"
WORKDIR "${BUILD_DIR}"

RUN ${APT_UPDATE} && ${APT_INSTALL} \
    \
    # System packages
    ca-certificates \
    \
    initramfs-tools=0.136ubuntu6.7 \
    linux-image-generic=5.4.0.150.148 \
    systemd-sysv=245.4-4ubuntu3.21 \
    \
    tcpreplay \
    \
    && ${APT_CACHE_PURGE}

RUN ${APT_UPDATE} && ${APT_INSTALL} \
    \
    iproute2 \
    pciutils \
    usbutils \
    dhcpcd5=7.1.0-2build1 \
    \
    && ${APT_CACHE_PURGE}

RUN ${APT_UPDATE} && ${APT_INSTALL} \
    \
    tcpdump \
    \
    && ${APT_CACHE_PURGE}

COPY ./2014-11-10-10-36-54_Velodyne-VLP_10Hz-County-Fair.pcap /root/velodyne.pcap

RUN tcprewrite -S 192.168.1.200:192.168.1.201 -i /root/velodyne.pcap -o /root/velodyne-re.pcap

RUN echo "root:root" | chpasswd
