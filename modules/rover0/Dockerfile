FROM ros:jazzy-ros-base-noble AS builder

ENV DEBIAN_FRONTEND=noninteractive

FROM ros:jazzy-ros-base-noble

RUN usermod -aG dialout root

RUN apt-get update

COPY ./entrypoint.sh /
ENTRYPOINT ["/bin/sh", "/entrypoint.sh"]
