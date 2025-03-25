FROM ros:jazzy-ros-base-noble AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y python3-pip git python3-jinja2 libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev cmake pkg-config python3-yaml python3-ply ninja-build

RUN pip3 install meson --break-system-packages

RUN <<EOF
cd
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled
ninja -C build
ninja -C build install
EOF

RUN apt install -y libjpeg-dev libpng-dev libboost-program-options-dev libexif-dev ninja-build

RUN <<EOF
cd
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
meson setup build -Denable_libav=disabled -Denable_drm=disabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
meson compile -C build
meson install -C build
EOF

FROM ros:jazzy-ros-base-noble

RUN usermod -aG video root

RUN apt update  && \
    apt install -y libjpeg8 libtiff6 libpng16-16 libexif12 libboost-program-options1.83.0 ffmpeg && \
    apt clean

COPY --from=builder /usr/local/bin/* /usr/local/bin/
COPY --from=builder /usr/local/share/libcamera /usr/local/share/libcamera
COPY --from=builder /usr/local/include/libcamera /usr/local/include/libcamera
COPY --from=builder /usr/local/share/libpisp /usr/local/share/libpisp
COPY --from=builder /usr/local/include/libpisp /usr/local/include/libpiso
COPY --from=builder /usr/local/share/rpi-camera-assets /usr/local/share/rpi-camera-assets
COPY --from=builder /usr/local/include/rpicam-apps /usr/local/include/rpicam-apps
COPY --from=builder /usr/local/include/libcamera-apps /usr/local/include/libcamera-apps
COPY --from=builder /usr/local/lib/aarch64-linux-gnu /usr/local/lib/aarch64-linux-gnu

RUN ldconfig

COPY ./entrypoint.sh /
ENTRYPOINT ["/bin/sh", "/entrypoint.sh"]
