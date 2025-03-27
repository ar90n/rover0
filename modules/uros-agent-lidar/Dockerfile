FROM ros:jazzy-ros-base-noble as base

ENV DEBIAN_FRONTEND=noninteractive
RUN usermod -aG dialout root

FROM base as dev

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./uros-agent-lidar/src,target=/tmp/modules/uros-agent-lidar/src \
    . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    apt-get update && \
    rosdep update && \
    rosdep install -y -r --from-paths /tmp/modules/uros-agent-lidar/src

ENTRYPOINT ["tail", "-f", "/dev/null"]


FROM dev as build

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./uros-agent-lidar/src,target=/tmp/modules/uros-agent-lidar/src \
    . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    cd /tmp/modules/uros-agent-lidar && \
    colcon build  --install-base /opt/rover0/


FROM base as final

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./uros-agent-lidar/src,target=/tmp/modules/uros-agent-lidar/src \
    . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    apt-get update && \
    rosdep update && \
    rosdep install -y -r --from-paths /tmp/modules/uros-agent-lidar/src -t exec

COPY --from=build /opt/rover0 /opt/rover0

COPY <<EOF /entrypoint.sh
#! /usr/bin/bash

source "/opt/ros/$ROS_DISTRO/setup.sh"
source "/opt/rover0/local_setup.sh"

exec ros2 run micro_ros_agent micro_ros_agent "\$@"
EOF

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
