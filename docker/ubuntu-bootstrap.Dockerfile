FROM alpine:3.18.2 AS livebuilder

# Install debian debootstrap utility from alpine/main
RUN apk add debootstrap

# Copy and install ubuntu-archive-keyring from host
COPY ./keys/ubuntu-archive-keyring.asc /tmp/ubuntu-archive-keyring.asc
RUN gpg \
    --keyring "/usr/share/keyrings/ubuntu-archive-keyring.gpg" \
    --no-default-keyring \
    --import "/tmp/ubuntu-archive-keyring.asc"

ARG DEB_MIRROR="http://archive.ubuntu.com/ubuntu/"
ARG DEB_SUITE="focal"
ARG DEB_COMPONENTS="main universe"

ARG DEB_CHROOT="/chroot"
ARG DEB_LIST="${DEB_CHROOT}/etc/apt/sources.list"

# Bootstrap ubuntu rootfs in chroot directory
RUN debootstrap \
    --force-check-gpg \
    --variant="minbase" \
    --arch="amd64" \
    "${DEB_SUITE}" \
    "${DEB_CHROOT}" \
    "${DEB_MIRROR}"

# Add extra suites (*-updates, *-security, etc.) and components (main, universe, etc.)
RUN rm "${DEB_LIST}" && \
    echo "deb ${DEB_MIRROR} ${DEB_SUITE}"          "${DEB_COMPONENTS}" >> "${DEB_LIST}" && \
    echo "deb ${DEB_MIRROR} ${DEB_SUITE}-updates"  "${DEB_COMPONENTS}" >> "${DEB_LIST}" && \
    echo "deb ${DEB_MIRROR} ${DEB_SUITE}-security" "${DEB_COMPONENTS}" >> "${DEB_LIST}"

# Purge default timezone config and apt cache
RUN rm -rf \
    "${DEB_CHROOT}/etc/localtime" \
    "${DEB_CHROOT}/etc/timezone" \
    "${DEB_CHROOT}/var/lib/apt/lists/*" \
    "${DEB_CHROOT}/var/cache/apt/*"

# Pivot via multi-stage build to scratch image, populated with chroot
FROM scratch
COPY --from=livebuilder /chroot /

# Configure user and build shell
USER  root
SHELL ["/usr/bin/env", "bash", "-o", "pipefail", "-c"]

# ====================================================================
# | Boilerplate Configuration
# ====================================================================

# Configure locales
ENV LANG="C.UTF-8"
ENV LC_ALL="C.UTF-8"

# Configure timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

# Prevent install of recommended or suggested packages (needed for rosdep)
RUN echo 'APT::Install-Recommends "0";' >> /etc/apt/apt.conf.d/95-no-recommend && \
    echo 'APT::Install-Suggests   "0";' >> /etc/apt/apt.conf.d/95-no-recommend

# Prevent generation of apt cache
RUN echo 'APT::Keep-Downloaded-Packages "false";' >> /etc/apt/apt.conf.d/95-no-cache && \
    echo 'Dir::Cache::srcpkgcache "";' >> /etc/apt/apt.conf.d/95-no-cache && \
    echo 'Dir::Cache::pkgcache    "";' >> /etc/apt/apt.conf.d/95-no-cache

# Block install of extraneous file paths to minimize image size
RUN echo 'path-exclude /usr/share/doc/*'           >> /etc/dpkg/dpkg.cfg.d/90-no-docs && \
    echo 'path-include /usr/share/doc/*/copyright' >> /etc/dpkg/dpkg.cfg.d/90-no-docs && \
    echo 'path-exclude /usr/share/man/*'           >> /etc/dpkg/dpkg.cfg.d/90-no-docs && \
    echo 'path-exclude /usr/share/groff/*'         >> /etc/dpkg/dpkg.cfg.d/90-no-docs && \
    echo 'path-exclude /usr/share/info/*'          >> /etc/dpkg/dpkg.cfg.d/90-no-docs && \
    echo 'path-exclude /usr/share/lintian/*'       >> /etc/dpkg/dpkg.cfg.d/90-no-docs && \
    \
    echo 'path-exclude /usr/share/locale/*'        >> /etc/dpkg/dpkg.cfg.d/90-no-locale && \
    \
    echo 'path-exclude /usr/share/fonts/*'         >> /etc/dpkg/dpkg.cfg.d/90-no-fonts

# Upgrade bootstrapped packages and get tls certs
RUN DEBIAN_FRONTEND="noninteractive" \
    DEBCONF_NOWARNINGS="yes" \
    apt-get update     && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    \
    ca-certificates \
    \
    && rm -rf /var/lib/apt/lists/*

CMD   ["/usr/bin/env", "bash"]
