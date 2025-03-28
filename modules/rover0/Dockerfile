FROM ros:jazzy-ros-base-noble as base

ENV DEBIAN_FRONTEND=noninteractive
RUN usermod -aG video root

FROM base AS dev

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./rover0/src,target=/tmp/modules/rover0/src \
    apt-get update && \
    rosdep update && \
    rosdep install -y -r --from-paths /tmp/modules/rover0/src -t build -t buildtool -t exec

ENTRYPOINT ["tail", "-f", "/dev/null"]

FROM dev as build

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./rover0/src,target=/tmp/modules/rover0/src \
    --mount=type=bind,source=./common,target=/tmp/modules/common \
    . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    cd /tmp/modules/rover0 && \
    colcon build  --install-base /opt/rover0/

FROM base as final

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./rover0/src,target=/tmp/modules/rover0/src \
    apt-get update && \
    . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    rosdep update && \
    rosdep install -y -r --from-paths /tmp/modules/rover0/src -t exec

COPY --from=build /opt/rover0 /opt/rover0

COPY <<EOF /entrypoint.sh
#! /bin/sh

. "/opt/ros/$ROS_DISTRO/setup.sh"
. "/opt/rover0/local_setup.sh"

exec ros2 launch rover0_bringup rover0.launch.py
EOF

ENTRYPOINT ["/bin/sh", "/entrypoint.sh"]
