# Base image
FROM docker.io/osrf/ros:jazzy-desktop-full
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1

## Install GTSAM into container

# Update packages and install system dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      build-essential cmake git apt-utils \
      libeigen3-dev libtbb-dev libboost-all-dev \
      python3 python3-venv python3-dev python3-pip \
      python3-pyqt6.sip && \
    rm -rf /var/lib/apt/lists/*

# Clone GTSAM
WORKDIR /src
RUN git clone --branch release/4.3a0 https://github.com/borglab/gtsam.git

# Build & install C++ core
WORKDIR /src/gtsam/build
RUN cmake -DCMAKE_BUILD_TYPE=Release \
          .. && \
    make -j"$(nproc)" && make install

# Ensure runtime can find libgtsam
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

