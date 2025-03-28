FROM ros:jazzy-ros-base-noble as base

ENV DEBIAN_FRONTEND=noninteractive
RUN usermod -aG video root

FROM base AS dev

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./teleop/src,target=/tmp/modules/teleop/src \
    apt-get update && \
    apt-get install -y ffmpeg python3-pip git python3-jinja2 libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev cmake pkg-config python3-yaml python3-ply ninja-build libjpeg-dev libpng-dev libboost-program-options-dev libexif-dev ninja-build && \
    rosdep update && \
    rosdep install -y -r --from-paths /tmp/modules/teleop/src --skip-keys libcamera -t build -t buildtool -t exec

RUN pip3 install meson --break-system-packages

RUN <<EOF
cd
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled
ninja -C build
ninja -C build install
EOF

RUN <<EOF
cd
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
meson setup build -Denable_libav=disabled -Denable_drm=disabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
meson compile -C build
meson install -C build
EOF

RUN ldconfig

ENTRYPOINT ["tail", "-f", "/dev/null"]

FROM dev as build

WORKDIR /tmp/modules

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./teleop/src,target=/tmp/modules/teleop/src \
    . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    cd /tmp/modules/teleop && \
    colcon build  --install-base /opt/rover0/

FROM base as final

RUN --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=bind,source=./teleop/src,target=/tmp/modules/teleop/src \
    apt-get update && \
    apt-get install -y \
      libgnutls30 \
      openssl \
      libjpeg8 \
      libtiff6 \
      libpng16-16 \
      libexif12 \
      libboost-program-options1.83.0 \
      ffmpeg  && \
    . "/opt/ros/$ROS_DISTRO/setup.sh" && \
    rosdep update && \
    rosdep install -y -r --from-paths /tmp/modules/teleop/src --skip-keys libcamera -t exec

COPY --from=build /usr/local/bin/* /usr/local/bin/
COPY --from=build /usr/local/share/libcamera /usr/local/share/libcamera
COPY --from=build /usr/local/include/libcamera /usr/local/include/libcamera
COPY --from=build /usr/local/share/libpisp /usr/local/share/libpisp
COPY --from=build /usr/local/include/libpisp /usr/local/include/libpisp
COPY --from=build /usr/local/share/rpi-camera-assets /usr/local/share/rpi-camera-assets
COPY --from=build /usr/local/include/rpicam-apps /usr/local/include/rpicam-apps
COPY --from=build /usr/local/include/libcamera-apps /usr/local/include/libcamera-apps
COPY --from=build /usr/local/lib/aarch64-linux-gnu /usr/local/lib/aarch64-linux-gnu
COPY --from=build /opt/rover0 /opt/rover0

RUN ldconfig

COPY <<EOF /entrypoint.sh
#! /bin/sh

. "/opt/ros/$ROS_DISTRO/setup.sh"
. "/opt/rover0/local_setup.sh"

exec ros2 launch teleop_bringup teleop.launch.py
EOF

ENTRYPOINT ["/bin/sh", "/entrypoint.sh"]
