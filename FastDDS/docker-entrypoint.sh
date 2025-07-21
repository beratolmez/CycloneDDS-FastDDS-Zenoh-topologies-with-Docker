#!/bin/bash
set -e

# ROS2 ortamını yükle
source /opt/ros/humble/setup.bash

# FastDDS konfigürasyonunu kontrol et
if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "FastDDS configuration file: $FASTRTPS_DEFAULT_PROFILES_FILE"
    if [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
        echo "Configuration file exists and is accessible"
    else
        echo "Warning: Configuration file not found or not accessible"
    fi
fi

# RMW implementation kontrolü
echo "RMW Implementation: $RMW_IMPLEMENTATION"

# Network bilgilerini göster
echo "Network Interface Information:"
ip addr show

echo "Container hostname: $(hostname)"
echo "Container IP addresses:"
hostname -I

# Verilen komutu çalıştır
exec "$@"
