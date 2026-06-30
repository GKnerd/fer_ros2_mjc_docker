#    Copyright 2025 Proximity Robotics & Automation GmbH
#    Modifications copyright 2026 Georgios Katranis
#
#    This Dockerfile is derived from internal Docker tooling developed at
#    Proximity Robotics & Automation GmbH. It has been adapted for the
#    Franka Emika Robot (FER) MuJoCo simulation use case: dependency set,
#    user-creation block, CycloneDDS configuration, and the colcon build
#    stage have been modified. The original layering pattern is retained.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ENV PROJECT_WS=ros2_ws
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################


RUN apt-get update && apt-get upgrade -y \
    && apt-get install --no-install-recommends -y \
    apt-utils \
    sudo \
    nano \
    curl \
    wget \
    build-essential \
    software-properties-common \
    cmake \
    git \
    gcc \
    libpoco-dev \
    libfmt-dev  \
    libeigen3-dev \
    python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=fer_ros2_sim
ARG UID=1000
ARG GID=$UID
ARG ROS_DOMAIN_ID=0
ARG VIDEO_GID=44
ENV USER=${USER}
ENV UID=${UID}
ENV GID=${GID}
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# Remove any existing user holding the target UID (e.g. Ubuntu Noble ships
# an `ubuntu` user at UID 1000) and recreate it under ${USER}. Hardware
# groups (video, dialout, plugdev, realtime) enable GUI/USB/serial access
# and SCHED_FIFO scheduling for ros2_control on the real robot later on.
RUN existing_user="$(getent passwd | awk -F: -v target_uid=${UID} '$3 == target_uid { print $1 }')" && \
    if [ -n "$existing_user" ]; then \
        userdel --remove "$existing_user"; \
    fi && \
    if ! getent group "$GID" >/dev/null 2>&1; then \
        groupadd -g "$GID" "$USER"; \
    fi && \
    useradd -m -u "$UID" -g "$GID" --shell "$(which bash)" "$USER" -G sudo && \
    groupadd realtime && \
    groupmod -g ${VIDEO_GID} video && \
    usermod -aG video "$USER" && \
    usermod -aG dialout "$USER" && \
    usermod -aG realtime "$USER" && \
    usermod -aG plugdev "$USER"
RUN echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp && \
    chmod 0440 /etc/sudoers.d/sudogrp

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc
RUN echo "source /home/$USER/$PROJECT_WS/install/setup.bash" >> /etc/bash.bashrc
RUN echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> /etc/bash.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /etc/bash.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///home/${USER}/env/cyclone_dds.xml

# Create base directories
USER ${USER}
RUN mkdir -p /home/${USER}/${PROJECT_WS}/src
##############################################################################
##                                 Dependencies                             ##
##############################################################################
USER root

RUN apt-get update && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-moveit-task-constructor-core \
    ros-${ROS_DISTRO}-moveit-task-constructor-msgs \
    ros-${ROS_DISTRO}-moveit-task-constructor-capabilities \
    ros-${ROS_DISTRO}-moveit-task-constructor-visualization \
    ros-${ROS_DISTRO}-behaviortree-cpp \
    && rm -rf /var/lib/apt/lists/*

USER ${USER}
##############################################################################
##                                 Build ROS                                ##
##############################################################################
WORKDIR /home/${USER}/${PROJECT_WS}
COPY --chown=$USER:$USER ${PROJECT_WS}/src /home/$USER/${PROJECT_WS}/src
  
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \ 
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cmake-clean-cache --merge-install

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/${PROJECT_WS}/install/setup.bash"' \
    /ros_entrypoint.sh

CMD ["/bin/bash"]

