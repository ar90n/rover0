#! /usr/bin/bash

. "/opt/ros/$ROS_DISTRO/setup.sh"

if [ "$ENV" = "build" ]; then
  echo "Running colcon build..."
  rosdep update
  rosdep install -y -r --from-paths src -t build -t buildtool
  colcon build
  exit 0
elif [ "$ENV" = "dev" ]; then
  echo "Running colcon build --symlink-install..."
  rosdep update
  rosdep install -y -r --from-paths src -t build -t buildtool -t exec
  colcon build --symlink-install
elif [ "$ENV" = "prod" ]; then
  echo "Production environment - no operation."
else
  echo "Undefined or unknown ENV value: $ENV"
  exit 1
fi

. "./install/local_setup.sh"

exec ros2 run micro_ros_agent micro_ros_agent "$@"
