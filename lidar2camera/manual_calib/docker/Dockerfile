FROM ubuntu:20.04

LABEL version="0.1" \
      description="Docker image for Pangolin"

ARG DEBIAN_FRONTEND=noninteractive
ENV PANGOLIN_VERSION="v0.8"

WORKDIR /tmp

# Install pangolin deps, opencv and pcl
RUN apt update_xyz_from_slider && \
    apt install -y python3-pip python3-dev ninja-build build-essential libeigen3-dev cmake git wget vim\
    libjpeg-dev libpng-dev libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
    libc++-dev libglew-dev libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols libegl1-mesa-dev \
    libpcl-dev libopencv-dev python3-opencv gdb zenity && \
    rm -rf /var/lib/apt/lists/*


# install pangolin
RUN git clone --recursive --depth 1 --branch ${PANGOLIN_VERSION} https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    cmake -B build -GNinja && \
    cmake --build build && \
    cmake --install build

ENV XDG_RUNTIME_DIR=/tmp/xdg_runtime_dir

WORKDIR /tmp

CMD ["bash"]
