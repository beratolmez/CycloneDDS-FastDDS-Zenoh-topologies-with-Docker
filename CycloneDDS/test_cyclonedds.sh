#!/bin/bash

echo "CycloneDDS Deney Düzeneği Test Script'i"
echo "========================================"

# Docker Compose dosyasının varlığını kontrol et
if [ ! -f "docker-compose.yml" ]; then
    echo "Hata: docker-compose.yml dosyası bulunamadı!"
    exit 1
fi

# Gerekli dosyaların varlığını kontrol et
echo "Gerekli dosyalar kontrol ediliyor..."

required_files=(
    "Dockerfile_ros2_cyclonedds"
    "docker-entrypoint-cyclonedds.sh"
    "cyclonedds-istanbul.xml"
    "cyclonedds-ankara.xml"
    "cyclonedds-izmir.xml"
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

# Docker Compose ile sistemi başlat
echo "Docker Compose ile sistem başlatılıyor..."
docker-compose up -d

# Servislerin çalışmasını bekle
echo "Servislerin başlaması bekleniyor (30 saniye)..."
sleep 30

# Container'ların durumunu kontrol et
echo "Container'ların durumu:"
echo "======================"
docker-compose ps

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

# CycloneDDS ayarlarını kontrol et
echo ""
echo "CycloneDDS Konfigürasyon Kontrolü:"
echo "=================================="

echo "İstanbul CycloneDDS ayarları:"
docker exec ist_terminal env | grep CYCLONEDDS 2>/dev/null || echo "CycloneDDS environment variables bulunamadı"

echo ""
echo "Ankara CycloneDDS ayarları:"
docker exec ankr_terminal env | grep CYCLONEDDS 2>/dev/null || echo "CycloneDDS environment variables bulunamadı"

echo ""
echo "İzmir CycloneDDS ayarları:"
docker exec izm_terminal env | grep CYCLONEDDS 2>/dev/null || echo "CycloneDDS environment variables bulunamadı"

# Inter-subnet iletişim testi
echo ""
echo "Inter-Subnet İletişim Testi:"
echo "============================"
echo "Test süresi: 10 saniye"

# Arka planda listener başlat
docker exec -d ankr_listener_1 timeout 15s ros2 topic echo /chatter --once 2>/dev/null

# Bir mesaj gönder
docker exec ist_talker_1 timeout 5s ros2 topic pub --once /chatter std_msgs/String "data: 'CycloneDDS Test mesajı Istanbul->Ankara'" 2>/dev/null && echo "Test mesajı gönderildi" || echo "Test mesajı gönderilemedi"

sleep 3

# CycloneDDS Daemon kontrolü
echo ""
echo "CycloneDDS Daemon Bilgileri:"
echo "============================"
echo "İstanbul CycloneDDS bilgileri:"
docker exec ist_terminal timeout 5s ros2 daemon status 2>/dev/null || echo "Daemon durumu alınamadı"

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
echo "   ros2 topic pub /chatter std_msgs/String \"data: 'CycloneDDS test mesajı'\""
echo ""
echo "5. CycloneDDS debug bilgileri için:"
echo "   export CYCLONEDDS_TRACE=trace"
echo "   ros2 node list"
echo ""
echo "6. Network connectivity testi için:"
echo "   ping 192.168.111.1  # İstanbul gateway"
echo "   ping 192.168.112.1  # Ankara gateway"
echo "   ping 192.168.113.1  # İzmir gateway"
echo ""
echo "7. Sistem durumunu görüntülemek için:"
echo "   docker-compose ps"
echo ""
echo "8. Logları görüntülemek için:"
echo "   docker-compose logs [service_name]"
echo ""
echo "9. Sistemi durdurmak için:"
echo "   docker-compose down"

echo ""
echo "CycloneDDS Test tamamlandı!"
