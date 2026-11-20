FROM ghcr.io/clearpathrobotics/clearpath_docker@sha256:970015d533d52c816d1d59ec10cfc2d5df20977b8d8d34c89c67f58ee6543315

USER root

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-clearpath-nav2-demos \
        ros-jazzy-behaviortree-cpp \
        ros-jazzy-nav2-behavior-tree \
        ros-jazzy-nav2-common \
        ros-jazzy-nav2-core \
        ros-jazzy-nav2-costmap-2d \
        ros-jazzy-nav2-lifecycle-manager \
        ros-jazzy-nav2-msgs \
        ros-jazzy-nav2-route \
        ros-jazzy-nav2-rviz-plugins \
        ros-jazzy-nav2-util \
        ros-jazzy-nav2-voxel-grid \
    && rm -rf /var/lib/apt/lists/*

USER robot

WORKDIR /home/robot