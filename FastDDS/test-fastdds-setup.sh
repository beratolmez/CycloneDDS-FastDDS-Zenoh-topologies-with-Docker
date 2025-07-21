#!/bin/bash

echo "FastDDS Deney Düzeneği Test Script'i"
echo "===================================="

# Docker Compose dosyasının varlığını kontrol et
if [ ! -f "docker-compose.yml" ]; then
    echo "Hata: docker-compose.yml dosyası bulunamadı!"
    exit 1
fi

# Gerekli dosyaların varlığını kontrol et
echo "Gerekli dosyalar kontrol ediliyor..."

required_files=(
    "Dockerfile_ros2_fastdds"
    "Dockerfile_fastdds_discovery"
    "docker-entrypoint.sh"
    "docker-entrypoint-discovery.sh"
    "scripts/start_discovery_server.sh"
    "fastdds-client-istanbul.xml"
    "fastdds-client-ankara.xml"
    "fastdds-client-izmir.xml"
    "fastdds-discovery-istanbul.xml"
    "fastdds-discovery-ankara.xml"
    "fastdds-discovery-izmir.xml"
)

missing_files=()
for file in "${required_files[@]}"; do
    if [ ! -f "$file" ]; then
        missing_files+=("$file")
    fi
done

if [ ${#missing_files[@]} -ne 0 ]; then
    echo "Eksik dosyalar:"
    for file in "${missing_files[@]}"; do
        echo "  - $file"
    done
    echo "Lütfen eksik dosyaları oluşturun."
    exit 1
fi

echo "Tüm gerekli dosyalar mevcut."

# Scripts dizinini oluştur
mkdir -p scripts

# Docker Compose ile sistemi başlat
echo "Docker Compose ile sistem başlatılıyor..."
docker-compose up -d

# Servislerin çalışmasını bekle
echo "Servislerin başlaması bekleniyor (30 saniye)..."
sleep 30

# Discovery Server'ların durumunu kontrol et
echo "Discovery Server'ların durumu:"
echo "=============================="
docker-compose ps | grep discovery_server

# Container'ların ağ durumunu kontrol et
echo ""
echo "Container Network Bilgileri:"
echo "============================"

echo "İstanbul Network:"
docker exec ist_talker_1 ip addr show 2>/dev/null | grep inet || echo "İstanbul container'ı bulunamadı"

echo "Ankara Network:"
docker exec ankr_talker_1 ip addr show 2>/dev/null | grep inet || echo "Ankara container'ı bulunamadı"

echo "İzmir Network:"
docker exec izm_talker_1 ip addr show 2>/dev/null | grep inet || echo "İzmir container'ı bulunamadı"

# ROS2 node'ların durumunu kontrol et
echo ""
echo "ROS2 Node Durumu Test Ediliyor:"
echo "==============================="

# İstanbul terminal'den diğer node'ları görebiliyor mu?
echo "İstanbul'dan ROS2 node listesi:"
docker exec ist_terminal timeout 10s ros2 node list 2>/dev/null || echo "ROS2 node listesi alınamadı"

echo ""
echo "Ankara'dan ROS2 node listesi:"
docker exec ankr_terminal timeout 10s ros2 node list 2>/dev/null || echo "ROS2 node listesi alınamadı"

echo ""
echo "İzmir'den ROS2 node listesi:"
docker exec izm_terminal timeout 10s ros2 node list 2>/dev/null || echo "ROS2 node listesi alınamadı"

# Topic listesini kontrol et
echo ""
echo "Aktif ROS2 Topic'ler:"
echo "===================="
docker exec ist_terminal timeout 10s ros2 topic list 2>/dev/null || echo "Topic listesi alınamadı"

# Discovery Server loglarını kontrol et
echo ""
echo "Discovery Server Logları:"
echo "========================"
echo "İstanbul Discovery Server:"
docker logs discovery_server_istanbul --tail 10 2>/dev/null || echo "İstanbul discovery server logları alınamadı"

echo ""
echo "Ankara Discovery Server:"
docker logs discovery_server_ankara --tail 10 2>/dev/null || echo "Ankara discovery server logları alınamadı"

echo ""
echo "İzmir Discovery Server:"
docker logs discovery_server_izmir --tail 10 2>/dev/null || echo "İzmir discovery server logları alınamadı"

# Inter-subnet iletişim testi
echo ""
echo "Inter-Subnet İletişim Testi:"
echo "============================"
echo "Test süresi: 10 saniye"

# Arka planda listener başlat
docker exec -d ankr_listener_1 timeout 15s ros2 topic echo /chatter --once 2>/dev/null

# Bir mesaj gönder
docker exec ist_talker_1 timeout 5s ros2 topic pub --once /chatter std_msgs/String "data: 'Test mesajı Istanbul->Ankara'" 2>/dev/null && echo "Test mesajı gönderildi" || echo "Test mesajı gönderilemedi"

sleep 3

# Manuel test talimatları
echo ""
echo "Manuel Test Talimatları:"
echo "======================="
echo "1. Terminal erişimi için:"
echo "   docker exec -it ist_terminal bash"
echo "   docker exec -it ankr_terminal bash"
echo "   docker exec -it izm_terminal bash"
echo ""
echo "2. Node'ları listelemek için:"
echo "   ros2 node list"
echo ""
echo "3. Topic'leri dinlemek için:"
echo "   ros2 topic echo /chatter"
echo ""
echo "4. Manuel mesaj göndermek için:"
echo "   ros2 topic pub /chatter std_msgs/String \"data: 'test mesajı'\""
echo ""
echo "5. Sistem durumunu görüntülemek için:"
echo "   docker-compose ps"
echo ""
echo "6. Logları görüntülemek için:"
echo "   docker-compose logs [service_name]"
echo ""
echo "7. Sistemi durdurmak için:"
echo "   docker-compose down"

echo ""
echo "Test tamamlandı!"
