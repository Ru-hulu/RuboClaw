FROM m.daocloud.io/docker.io/library/ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i \
        -e 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
        -e 's|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' \
        /etc/apt/sources.list.d/ubuntu.sources \
    && sed -i \
        -e 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
        -e 's/^Types: deb deb-src$/Types: deb/' \
        /etc/apt/sources.list.d/ros2.sources \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-pip \
        python3-venv \
        ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/RoboClaw

CMD ["bash"]
