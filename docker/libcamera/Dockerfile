FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y python3-pip git python3-jinja2 libboost-dev libgnutls28-dev openssl libtiff5-dev pybind11-dev cmake pkg-config python3-yaml python3-ply ninja-build

RUN pip3 install meson

RUN <<EOF
cd
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled
ninja -C build
ninja -C build install
EOF

RUN apt install -y libjpeg-dev libpng-dev libboost-program-options1.74-dev libexif-dev ninja-build

RUN <<EOF
cd
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
meson setup build -Denable_libav=false -Denable_drm=false -Denable_egl=false -Denable_qt=false -Denable_opencv=false -Denable_tflite=false
meson compile -C build
meson install -C build
EOF

FROM ubuntu:22.04
RUN apt update  && \
    apt install -y libjpeg8 libtiff5 libpng16-16 libexif12 libboost-program-options1.74.0 && \
    apt clean

COPY --from=builder /usr/local/bin/* /usr/local/bin/
COPY --from=builder /usr/local/share/libcamera /usr/local/share/libcamera
COPY --from=builder /usr/local/share/libpisp /usr/local/share/libpisp
COPY --from=builder /usr/local/lib/aarch64-linux-gnu /usr/local/lib/aarch64-linux-gnu

RUN ldconfig
