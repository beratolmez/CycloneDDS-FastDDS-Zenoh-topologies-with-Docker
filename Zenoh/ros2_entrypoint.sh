#!/bin/bash
set -e

# ROS ortamını yükle
source /opt/ros/humble/setup.bash

# Kullanıcının kendi workspace'i varsa onu da source edebilirsiniz
# source /ros2_ws/install/setup.bash

# Eğer interaktif shell açılıyorsa .bashrc çalışsın
if [ -n "$PS1" ]; then
  exec bash --rcfile <(echo 'source /opt/ros/humble/setup.bash; exec bash')
else
  exec "$@"
fi

