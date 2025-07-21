#!/bin/bash
set -e

# ROS2 ortamını yükle
source /opt/ros/humble/setup.bash

# Network bilgilerini göster
echo "Discovery Server Network Information:"
ip addr show
echo "Container hostname: $(hostname)"
echo "Container IP addresses:"
hostname -I

# FastDDS Discovery Server parametrelerini göster
echo "FastDDS Discovery Server Configuration:"
echo "Server ID: $FASTDDS_SERVER_ID"  
echo "Server Port: $FASTDDS_SERVER_PORT"

# Discovery Server IP'lerini göster
echo "Discovery Server IPs:"
echo "Istanbul: 172.30.0.2"
echo "Ankara: 192.168.112.2" 
echo "Izmir: 172.30.0.3"

# Konfigürasyon dosyasını kontrol et
if [ -f "/config/fastdds-discovery.xml" ]; then
    echo "Using configuration file: /config/fastdds-discovery.xml"
    export FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds-discovery.xml
else
    echo "Warning: No configuration file found, using default settings"
fi

# Fast-discovery-server aracının var olup olmadığını kontrol et
if command -v fast-discovery-server &> /dev/null; then
    echo "fast-discovery-server tool found!"
    
    # Eğer discovery server başlatma komutu verilmişse
    if [[ "$1" == "start-discovery" ]]; then
        echo "Starting FastDDS Discovery Server..."
        echo "Server ID: $FASTDDS_SERVER_ID"
        echo "Server Port: $FASTDDS_SERVER_PORT"
        
        # Doğru parametrelerle discovery server'ı başlat
        exec fast-discovery-server -i $FASTDDS_SERVER_ID -p $FASTDDS_SERVER_PORT
    fi
else
    echo "fast-discovery-server tool not found!"
    echo "Checking alternative commands..."
    
    # Alternatif komutları dene
    if command -v fastdds &> /dev/null; then
        echo "fastdds command found, trying discovery server..."
        if [[ "$1" == "start-discovery" ]]; then
            # FastDDS discovery server komutunu doğru parametrelerle çalıştır
            exec fastdds discovery -i $FASTDDS_SERVER_ID -p $FASTDDS_SERVER_PORT
        fi
    else
        echo "No FastDDS discovery server tools found!"
        echo "Available commands:"
        ls -la /usr/bin/ | grep -i fast || echo "No fast commands found"
        ls -la /usr/local/bin/ | grep -i fast || echo "No fast commands in /usr/local/bin"
    fi
fi

# Verilen komutu çalıştır
exec "$@"
