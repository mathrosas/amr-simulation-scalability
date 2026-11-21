FROM ghcr.io/clearpathrobotics/clearpath_docker@sha256:970015d533d52c816d1d59ec10cfc2d5df20977b8d8d34c89c67f58ee6543315

USER root

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        --no-install-recommends \
        ros-jazzy-rmf-dev \
        ros-jazzy-action-msgs \
    && rm -rf /var/lib/apt/lists/*

USER robot

WORKDIR /home/robot

CMD ["/bin/bash"]