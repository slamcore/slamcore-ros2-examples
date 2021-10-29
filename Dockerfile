ARG version=foxy
FROM ros:${version}-ros-core
ARG version

# Enable GPU access
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:-all}

ARG SLAMCORE_DEB

# Install bootstrap tools and dev utils
RUN apt-get update && \
    DEBIAN_FRONTEND="noninteractive" \
    apt-get install -q -y --no-install-recommends \
    apt-utils \
    bash-completion \
    build-essential \
    curl \
    dirmngr \
    git \
    git \
    gnupg2 \
    htop \
    libcurl4-openssl-dev \
    python3 \
    python3-pip \
    sudo \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# pip3 dependencies
RUN pip3 install wheel && \
    pip3 install pyyaml rospkg toml && \
    pip3 install "git+https://github.com/dirk-thomas/vcstool"

# ROS packages
RUN apt-get update && \
    DEBIAN_FRONTEND="noninteractive" \
    apt-get install -y --no-install-recommends \
    keyboard-configuration \
    python3-colcon-* \
    python3-rosdep \
    ros-${version}-diagnostic-updater \
    ros-${version}-ecl-build \
    ros-${version}-joint-state-publisher \
    ros-${version}-kobuki* \
    ros-${version}-nav2* \
    ros-${version}-navigation2* \
    ros-${version}-nonpersistent-voxel-layer \
    ros-${version}-ros-base \
    ros-${version}-rviz2 \
    ros-${version}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Setup default user
ARG USERNAME=slamcore
ARG ADDITIONAL_GROUPS
ARG UID
ARG GID
ARG HOME=/home/$USERNAME
ARG ROS_WS=$HOME/ros_ws
ENV ROS_WS=$ROS_WS

# Install the SLAMcore ROS2 Package
COPY $SLAMCORE_DEB /slamcore_ros.deb
RUN apt-get update && \
    DEBIAN_FRONTEND="noninteractive" \
    apt-get install -y --no-install-recommends \
    /slamcore_ros.deb

COPY /add-groups.sh /add-groups.sh
RUN /add-groups.sh $ADDITIONAL_GROUPS

RUN groupadd -g $GID -o $USERNAME
RUN useradd -m -u $UID -g $GID --groups sudo,$ADDITIONAL_GROUPS -p $(openssl passwd -1 $USERNAME) $USERNAME
RUN usermod -aG sudo $USERNAME
RUN echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME
RUN cat /etc/sudoers.d/$USERNAME

# Create a sample ROS workspace that the user can mount and build from source
# their ROS packages
RUN mkdir $ROS_WS
RUN chown $USERNAME:$USERNAME $ROS_WS

COPY /entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
WORKDIR $ROS_WS

# As user ----------------------------------------------------------------------
USER $USERNAME
ENV USER=$USERNAME

RUN sudo rosdep init && rosdep update
