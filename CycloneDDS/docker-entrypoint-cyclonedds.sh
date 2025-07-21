#!/bin/bash
set -e

# ROS2 ortamını yükle
source /opt/ros/humble/setup.bash

# CycloneDDS konfigürasyonunu kontrol et
if [ -n "$CYCLONEDDS_URI" ]; then
    echo "CycloneDDS configuration file: $CYCLONEDDS_URI"
    CONFIG_FILE=$(echo $CYCLONEDDS_URI | sed 's/file:\/\///')
    if [ -f "$CONFIG_FILE" ]; then
        echo "Configuration file exists and is accessible"
        echo "Configuration file size: $(wc -c < $CONFIG_FILE) bytes"
    else
        echo "Warning: Configuration file not found or not accessible"
    fi
fi

# RMW implementation kontrolü
echo "RMW Implementation: $RMW_IMPLEMENTATION"

# Domain tag kontrolü
if [ -n "$CYCLONEDDS_DOMAIN_TAG" ]; then
    echo "CycloneDDS Domain Tag: $CYCLONEDDS_DOMAIN_TAG"
fi

# ROS Domain ID kontrolü
echo "ROS Domain ID: $ROS_DOMAIN_ID"

# Network bilgilerini göster
echo "Network Interface Information:"
ip addr show

echo "Container hostname: $(hostname)"
echo "Container IP addresses:"
hostname -I

# CycloneDDS debug bilgileri için environment variable'lar
echo "CycloneDDS Environment Variables:"
env | grep -E "(CYCLONEDDS|RMW)" || echo "No CycloneDDS specific environment variables found"

# Verilen komutu çalıştır
exec "$@"
