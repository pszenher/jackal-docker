ARG DEB_SUITE="focal"
FROM pszenher/ubuntu-bootstrap:${DEB_SUITE}


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
RUN echo 'APT::Install-Recommends "0";' >> /etc/apt/apt.conf.d/99-no-recommend && \
    echo 'APT::Install-Suggests   "0";' >> /etc/apt/apt.conf.d/99-no-recommend

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
