# FROM osrf/ros:humble-desktop-full
# ARG USERNAME=mobile
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID

# # Create the user
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     #
#     # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
#     && apt-get update \
#     && apt-get install -y sudo \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME
# RUN apt-get update && apt-get upgrade -y
# RUN apt-get install -y python3-pip
# RUN apt-get install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-ros2-controllers ros-rolling-ros2-control ros-humble-rosbridge-suite --fix-missing
# ENV SHELL /bin/bash
# # ********************************************************
# # * Anything else you want to do like clean up goes here *
# # ********************************************************

# # [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME
# CMD ["/bin/bash"]

FROM osrf/ros:humble-desktop-full
ARG USERNAME=mobile
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get update && apt-get install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-msgs ros-humble-ros2-controllers ros-humble-rosbridge-suite ros-rolling-ros2-control

# RUN apt-get update && apt-get install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-ros2-controllers ros-humble-rosbridge-suite ros-rolling-ros2-control
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]
