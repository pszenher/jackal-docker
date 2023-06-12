FROM ubuntu:focal

ARG APT_UPDATE="apt-get -qq update"
ARG APT_INSTALL="apt-get -qq install --no-install-recommends -y"
ARG APT_CACHE_PURGE="rm -rf /var/lib/apt/lists/*"

ARG GIT_CLONE="git clone --quiet --depth=1 --single-branch --config advice.detachedHead=false"

ARG BUILD_DIR="/tmp"
WORKDIR "${BUILD_DIR}"

RUN ${APT_UPDATE} && ${APT_INSTALL} \
    \
    # System packages
    ca-certificates \
    git \
    sudo \
    \
    && ${APT_CACHE_PURGE}

RUN ${GIT_CLONE} \
    "https://github.com/intrig-unicamp/mininet-wifi.git" \
    "${BUILD_DIR}/mininet-wifi"
    # --commit "8788bf3b1947053f1da79a806472ca0773136cd5"

WORKDIR "${BUILD_DIR}/mininet-wifi"

RUN ${APT_UPDATE} && \
    util/install.sh -Wlnfv
