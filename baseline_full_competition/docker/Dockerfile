# syntax=docker/dockerfile:1.7

FROM nvidia/cuda:12.8.0-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
ARG LIBTORCH_URL=https://download.pytorch.org/libtorch/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu128.zip
ARG USER_NAME=simenv
ARG USER_UID=1000
ARG USER_GID=1000

ENV TZ=Asia/Shanghai
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LIBTORCH_ROOT=/opt/libtorch
ENV TORCH_CUDA_ARCH_LIST=12.0
ENV CMAKE_PREFIX_PATH=/opt/libtorch
ENV LD_LIBRARY_PATH=/opt/libtorch/lib:/usr/local/cuda/lib64:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Basic build and repository tools.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gnupg2 \
        locales \
        lsb-release \
        pkg-config \
        software-properties-common \
        tzdata \
        unzip \
        wget \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Official ROS package repository.
RUN curl -fsSL \
        https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
        > /etc/apt/sources.list.d/ros1.list

# ROS Noetic, Gazebo Classic and common project dependencies.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full \
        python3-catkin-tools \
        python3-numpy \
        python3-opencv \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        python3-yaml \
        libarmadillo-dev \
        libeigen3-dev \
        liblcm-dev \
        libyaml-cpp-dev \
        mesa-utils \
        ninja-build \
        qtbase5-dev \
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep. Noetic is EOL, so it must be requested explicitly.
RUN rosdep init \
    && rosdep update \
        --include-eol-distros \
        --rosdistro=${ROS_DISTRO}

# Resolve dependencies declared by the repository without copying source
# files into the final image.
RUN --mount=type=bind,source=src,target=/tmp/simenv-src,readonly \
    apt-get update \
    && rosdep install \
        --from-paths /tmp/simenv-src \
        --ignore-src \
        --rosdistro=${ROS_DISTRO} \
        --reinstall \
        -y \
    && rm -rf /var/lib/apt/lists/*

# Download resumable LibTorch CUDA 12.8 package and install it in /opt.
RUN --mount=type=cache,target=/var/cache/libtorch \
    set -eux \
    && archive=/var/cache/libtorch/libtorch-cu128.zip \
    && wget \
        --continue \
        --tries=20 \
        --timeout=60 \
        --output-document="${archive}" \
        "${LIBTORCH_URL}" \
    && unzip -tq "${archive}" \
    && rm -rf /opt/libtorch \
    && unzip -q "${archive}" -d /opt \
    && test -f /opt/libtorch/share/cmake/Torch/TorchConfig.cmake

# Run development containers as a non-root user.
RUN groupadd --gid "${USER_GID}" "${USER_NAME}" \
    && useradd \
        --uid "${USER_UID}" \
        --gid "${USER_GID}" \
        --create-home \
        --shell /bin/bash \
        "${USER_NAME}"

COPY docker/entrypoint.sh /usr/local/bin/simenv-entrypoint
RUN chmod 0755 /usr/local/bin/simenv-entrypoint

USER ${USER_NAME}
WORKDIR /workspace/SimEnv

ENTRYPOINT ["/usr/local/bin/simenv-entrypoint"]
CMD ["bash"]
