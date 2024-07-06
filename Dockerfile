FROM osrf/ros:humble-desktop-full
ARG USERNAME=mobile
ARG USER_UID=1000
ARG USER_GID=$USER_UID

WORKDIR /home/ws/src
COPY . /home/ws/src

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN chown -R $USERNAME:$USERNAME /home/ws/src
RUN apt-get update && apt-get upgrade -y && apt-get install -y python3-pip ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rqt-robot-steering ros-humble-ros-gz --fix-missing

ENV DISPLAY unix:0
ENV ROS_AUTOMATIC_DISCOVERY_RANGE LOCALHOST
ENV ROS_DOMAIN_ID 42
ENV PATH $PATH:/home/$USERNAME/.local/bin
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
COPY startup.bash .

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["./startup.bash"]
