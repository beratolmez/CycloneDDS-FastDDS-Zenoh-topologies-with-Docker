#!/bin/bash
set -e

# ROS2 ortamını yükle
source /opt/ros/humble/setup.bash

echo "Starting FastDDS Discovery Server..."
echo "Server ID: $FASTDDS_SERVER_ID"
echo "Server Port: $FASTDDS_SERVER_PORT"

# Konfigürasyon dosyası varsa kullan
CONFIG_ARG=""
if [ -f "/config/fastdds-discovery.xml" ]; then
    echo "Using configuration file: /config/fastdds-discovery.xml"
    # -x parametresi ile XML dosyasını belirtin
    CONFIG_ARG="-x /config/fastdds-discovery.xml"
fi

# RMW_IMPLEMENTATION ayarla (Discovery server için de gerekebilir)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Executing: fast-discovery-server -i $FASTDDS_SERVER_ID -p $FASTDDS_SERVER_PORT $CONFIG_ARG"

# FastDDS Discovery Server'ı başlat
# Ortam değişkenlerinin doğru şekilde kullanıldığından emin olun
exec fast-discovery-server -i $FASTDDS_SERVER_ID -p $FASTDDS_SERVER_PORT $CONFIG_ARG
