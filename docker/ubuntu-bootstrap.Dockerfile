FROM alpine:3.18.2 AS livebuilder

# Install debian debootstrap utility from alpine/main
RUN apk add debootstrap

# Install ubuntu dist keyring from alpine/testing
RUN apk add ubuntu-archive-keyring \
    --repository=https://dl-cdn.alpinelinux.org/alpine/edge/testing

ARG DEB_MIRROR="http://archive.ubuntu.com/ubuntu/"
# ARG DEB_MIRROR="https://mirrors.wikimedia.org/ubuntu"
# ARG DEB_MIRROR="http://mirrors.rit.edu/ubuntu/"
ARG DEB_SUITE="focal"
ARG DEB_COMPONENTS="main,universe"

# Bootstrap ubuntu rootfs in chroot directory
RUN debootstrap \
    --force-check-gpg \
    --variant="minbase" \
    --arch="amd64" \
    # --components="${DEB_COMPONENTS}" \
    # --extra-suites="${DEB_SUITE}-updates" \
    "${DEB_SUITE}" \
    /chroot \
    "${DEB_MIRROR}"

# Add extra suites (*-updates, *-security, etc.) and components (main, universe, etc.)
COPY ./focal/minimal-sources.list /chroot/etc/apt/sources.list

# Purge default timezone config and apt cache
RUN rm -rf \
    chroot/etc/localtime \
    chroot/etc/timezone \
    chroot/var/lib/apt/lists/*

# Pivot via multi-stage build to scratch image, populated with chroot
FROM scratch
COPY --from=livebuilder /chroot /
CMD ["/bin/bash"]
