FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y git curl build-essential lsb-release gnupg gnupg2

# Add ROS2 apt repository
RUN <<EOF
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
EOF

# Install ROS2 packages
RUN apt update && \
    apt install -y ros-humble-ros-base python3-vcstool python3-rosdep python3-rosinstall python3-argcomplete python3-colcon-common-extensions

# Setup ROS environment
RUN rosdep init && rosdep update

# Add custom profile script
RUN cat <<EOF > /etc/profile.d/90-setup-ros2.sh
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
EOF
RUN chmod +x /etc/profile.d/90-setup-ros2.sh

# Clean up
RUN rm -rf /var/lib/apt/lists/*
