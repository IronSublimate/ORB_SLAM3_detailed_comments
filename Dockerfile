# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-core-focal

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools
#    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
#RUN rosdep init && \
#  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1*
#    && rm -rf /var/lib/apt/lists/*

# install project dependence
RUN  apt-get update && apt-get install -y --no-install-recommends \
    libfmt-dev  \
    libopencv-dev \
    gdb \
    git
#    && rm -rf /var/lib/apt/lists/*

# install g2o
RUN  apt-get update && apt-get install -y --no-install-recommends \
    qt5-default \
    libopenblas-dev \
    libeigen3-dev
RUN cd /tmp \
    && git clone https://github.com/RainerKuemmerle/g2o.git \
    && cd g2o \
    && git checkout 20201223_git \
    && mkdir build \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DG2O_USE_OPENMP=ON -DBUILD_WITH_MARCH_NATIVE=ON \
    && cmake --build build --target install -j 10 \
    && rm -rf /tmp/g2o

# install sophus
RUN cd /tmp \
    && git clone https://github.com/strasdat/Sophus.git \
    && cd Sophus \
    && git checkout 1.22.10 \
    && mkdir build \
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_EXAMPLES=OFF -DBUILD_SOPHUS_TESTS=OFF\
    && cmake --build build --target install -j 10 \
    && rm -rf /tmp/Sophus

# install pangolin
RUN cd /tmp && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
RUN cd /tmp/Pangolin \
    && git checkout v0.8 \
    && apt-get install -y --no-install-recommends  \
    libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev \
    ninja-build libglew-dev\
    && cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_EXAMPLES=OFF -DBUILD_SOPHUS_TESTS=OFF\
    && cmake --build build -t install -j 10 \
    && rm -rf /tmp/Sophus

RUN apt install -y --no-install-recommends \
    ros-noetic-cv-bridge