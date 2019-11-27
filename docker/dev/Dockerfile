
FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        sudo \
        ca-certificates \
        pkg-config \
        git \
        build-essential \
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
        doxygen \
        graphviz \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# install ros2 and dependencies (but not colcon!)
ARG CHOOSE_ROS_DISTRO=eloquent
ENV CHOOSE_ROS_DISTRO=$CHOOSE_ROS_DISTRO

RUN cd / \
    && wget https://github.com/ros2/ros2/releases/download/release-eloquent-20191122/ros2-eloquent-20191122-linux-bionic-amd64.tar.bz2 \
    && tar -xf ros2-eloquent-20191122-linux-bionic-amd64.tar.bz2 \
    && rm ros2-eloquent-20191122-linux-bionic-amd64.tar.bz2 \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update \
    && apt-get install -y python-rosdep \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths /ros2-linux/share --ignore-src --rosdistro $CHOOSE_ROS_DISTRO -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers" \
    && chmod -R go+w /ros2-linux \
    && echo "/ros2-linux/lib" > /etc/ld.so.conf.d/ros2.conf

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
