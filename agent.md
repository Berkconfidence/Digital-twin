# Proje Özeti: Çift Yönlü Dijital İkiz (Two-Way Digital Twin)

Bu proje, CARLA Simülatörü ve ROS 2 (Jazzy) kullanılarak geliştirilen bir **Çift Yönlü Dijital İkiz (Two-Way Digital Twin)** uygulamasıdır. `digital_twin_pkg` ROS 2 paketi altında geliştirilmektedir. Simülasyon dünyasındaki (CARLA) veriler sensörler aracılığıyla ROS 2 ortamına (gerçek dünya analizi senaryosu) aktarılırken, ROS 2 üzerinde çalışan algoritmik mantıklar ve kontrolcü komutları simülatördeki aracı ve ortamı otonom veya manuel olarak yönlendirmektedir.

Bu dosya, projeye yeni dahil olan veya kod yapısını anlamak isteyen bir yapay zeka asistanı veya geliştirici için temel rehber niteliğindedir. Aşağıda projeyi oluşturan temel kaynak kodları, veri yapıları ve modeller detaylı olarak özetlenmiştir.

## Sistem Özellikleri & Teknolojiler
- **İşletim Sistemi:** Linux Ubuntu 24.04
- **ROS Sürümü:** ROS 2 Jazzy
- **Simülatör:** CARLA (Senkron - Synchronous modda çalışmaktadır)
- **Derin Öğrenme Modeli:** YOLO11 (`yolo11_carla_train/weights/best.pt` ağırlıkları ile araç, yaya, çukur ve trafik lambası tespiti yapmaktadır)
- **Görselleştirme:** Foxglove Studio (`foxglove_bridge` üzerinden) ve OpenCV pencereleri.

---

## Projedeki Temel Kaynak Kodları ve Dosyalar

### 1. Düğümler (Nodes) ve Betikler

* **`advanced_perception_node.py`** (Gelişmiş Algılama ve Karar Düğümü)
  Projenin beyni konumundaki yapay zeka düğümüdür. `message_filters.ApproximateTimeSynchronizer` kullanarak ön RGB ve Derinlik (Depth) kameralarını senkronize eder. YOLO11 modelini çalıştırarak nesneleri tespit eder. 
  * *Çalışma Mantığı:* Tespit edilen piksellerin derinlik verisini alarak `ego_path_math.py` yardımıyla 3B kuşbakışı (BEV) koordinatlara çevirir. Yaya ve çukurların, aracın direksiyon açısına göre hesaplanan "Risk Koridoru (Ego-Path)" içinde olup olmadığını poligon testleriyle denetler. Trafik lambalarının (Kırmızı/Yeşil) uzaklığını hesaplar.
  * *Hız Kontrol Hiyerarşisi:* Tespitlere göre araca hız komutları gönderir. Öncelik sırası: Kritik Yaya (0 km/h) > Kırmızı Işık (0 km/h) > Çukur (5 km/h) > Normal Seyir (35 km/h) şeklindedir. Kesintileri önlemek için bekleme (cooldown) süreleri içerir.

* **`spawn_pedestrians.py`** (Yapay Zeka Yaya Doğurucu)
  CARLA dünyasında gelişmiş AI güdümlü yayalar oluşturan scripttir. 
  * *Önemli Detay:* CARLA senkron modda çalıştığı için, yayaların gerçekten hareket edebilmesi adına `rclpy.spin()` ile birlikte çalışan özel bir döngüde sürekli `world.wait_for_tick()` çağırır. Yayaların karşıdan karşıya geçme veya koşma oranları gibi zorlayıcı test parametreleri içerir.

* **`spawn_potholes_node.py`** (Çukur ve Enkaz Doğurucu)
  Yol hasarı/çukur algılama (pothole detection) özelliklerini test edebilmek için asfalta statik bozukluklar ekler. 
  * *Özellikleri:* Haritadaki waypoint'leri kullanarak yollara rastgele `dirtdebris` (moloz/toprak) ve `brokentile` (kırık fayans) gibi CARLA prop'ları yerleştirir. Prop boyutlarını ayarlayarak (örn. `size` attribute'unu değiştirerek) sistemin yapay zeka kamerasında belirgin, fiziksel olarak da aracın üzerinden geçerken sarsılacağı gerçekçi yol bozuklukları simüle eder.

* **`ego_path_math.py`** (Matematik ve Geometri Araçları)
  Algılama düğümünün (perception) mekansal farkındalığını sağlayan yardımcı modüldür.
  * *Fonksiyonları:* Kameradan gelen 2D piksel X koordinatını ve Derinlik Z verisini alarak, 3D Kuşbakışı (Bird's Eye View - BEV) X, Y koordinatlarına çeviren `get_bev_coordinates` fonksiyonunu barındırır. Ayrıca aracın anlık direksiyon açısına (steer) göre aracın gelecekteki rotasını çizen bir çokgen (polygon) oluşturan `get_ego_path_polygon` fonksiyonunu içerir.

### 2. Başlatıcılar ve Konfigürasyonlar (Launch & Config)

* **`digital_twin.launch.py`** (Ana Simülasyon Başlatıcısı)
  CARLA simülasyonunu ve ROS 2 köprülerini ayağa kaldıran ana omurgadır.
  * *İçerdikleri:* CARLA ortamını, `carla_ros_bridge`'i (senkron modda), aracı ve sensörleri (`carla_spawn_objects`), otonom sürüş rota planlayıcısını (`local_planner`), Foxglove görselleştirme köprüsünü (`foxglove_bridge`) ve aracın fiziksel URDF modelini (`robot_state_publisher`) tek bir komutla başlatır. Deney düğümleri (perception, spawner'lar) modülerliği korumak için genellikle ayrı terminallerden manuel çalıştırılır.

* **`objects.json`** (Araç ve Sensör Konfigürasyonu)
  Simülasyona eklenecek olan Ego aracı (Tesla Model 3) ve üzerine monte edilen sensörlerin kesin konumlarını, rotasyonlarını ve teknik özelliklerini tanımlar.
  * *Sensörler:* Ön RGB, Geniş Açılı RGB, Sol/Sağ/Arka Görüş kameraları, Derinlik (Depth) Kamerası, Semantik Segmentasyon Kamerası, Lidar (Tepede 64 kanal), Ön Radar, IMU, GNSS, Çarpışma ve Şerit İhlali gibi çok çeşitli donanımları içerir. Bu dosya, `digital_twin.launch.py` tarafından okunur.

* **`digital_twin_layout.json`** (Foxglove Studio Arayüz Şablonu)
  Telemetri ve görselleştirme aracı olan Foxglove Studio için hazırlanmış dashboard düzenidir.
  * *İçerdikleri:* Aracın Lidar, Radar, Odometry ve Waypoint verilerini 3 boyutlu izlemek için "3D Overview" paneli; gaz, fren, direksiyon, hız ve IMU grafiklerini çizen Plot panelleri; ve birden fazla kameranın (RGB, Semantic vb.) görüntülerini aynı anda izlemeye yarayan Image View panellerini hazır olarak sunar.

---

## Geliştirme Prensibi
Sistem son derece **modüler** tasarlanmıştır. Başlatıcılar (`launch` dosyaları) sadece stabil çekirdek sistemi ayağa kaldırırken, test edilen veya geliştirilen yapay zeka düğümleri (`advanced_perception_node.py` gibi) ve ortam manipülatörleri (`spawn_potholes_node.py` gibi) her zaman izole olarak kendi CLI komutlarıyla (örn. `ros2 run digital_twin_pkg ...`) çalıştırılarak test edilir. Simülatördeki her aktör, gerçek dünyadaki bir donanımın dijital karşılığı olarak (Digital Twin mantığıyla) ROS 2'ye veri basmaktadır.
