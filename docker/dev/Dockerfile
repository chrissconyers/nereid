
FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        sudo \
        ca-certificates \
        pkg-config \
        git \
        build-essential \
        gdb \
        ssh \
        rsync \
        wget \
        vim \
        less \
        net-tools \
        gettext-base \
        netbase \
        iputils-ping \
        curl \
        gnupg2 \
        lsb-release \
        locales \
        doxygen \
        graphviz \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# install ros2 and dependencies (but not colcon!)
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update \
    && apt-get install -y ros-eloquent-desktop \
    && echo "/opt/ros/eloquent/lib" > /etc/ld.so.conf.d/ros2.conf \
    && ldconfig

# install a newer version of cmake
ARG cmake_version=3.14.6
RUN mkdir /cmake \
    && cd /cmake \
    && wget https://github.com/Kitware/CMake/releases/download/v$cmake_version/cmake-$cmake_version.tar.gz \
    && tar -xf cmake-$cmake_version.tar.gz \
    && cd cmake-$cmake_version \
    && ./bootstrap \
    && make install \
    && rm -rf /cmake

# install nlohmann json
ARG nlohmann_json_version=456478b3c50d
RUN git clone https://github.com/nlohmann/json.git /json \
    && mkdir /json/build \
    && cd /json/build \
    && git checkout $nlohmann_json_version \
    && cmake .. -DBUILD_TESTING=OFF \
    && make install \
    && cd / \
    && rm -rf /json

# install eigen
ARG eigen_version=3.3.5
RUN git clone --branch $eigen_version https://github.com/eigenteam/eigen-git-mirror.git /eigen \
    && mkdir /eigen/build \
    && cd /eigen/build \
    && cmake .. \
    && make install \
    && cd / \
    && rm -rf /eigen

# install fmt
ARG fmt_version=6.0.0
RUN git clone --branch $fmt_version https://github.com/fmtlib/fmt.git /fmt \
    && mkdir /fmt/build \
    && cd /fmt/build \
    && cmake .. -DFMT_TEST=OFF \
    && make install \
    && cd / \
    && rm -rf /fmt
