# Proje Özeti: Çift Yönlü Dijital İkiz (Two-Way Digital Twin)

Bu proje, CARLA Simülatörü ve ROS 2 (Jazzy) kullanılarak geliştirilen bir **Çift Yönlü Dijital İkiz (Two-Way Digital Twin)** uygulamasıdır. Proje `digital_twin_pkg` ROS 2 paketi altında geliştirilmektedir. Simülasyon dünyasındaki (CARLA) veriler sensörler aracılığıyla ROS 2 ortamına (gerçek dünya analizi senaryosu) aktarılırken, ROS 2 üzerinde çalışan algoritmik mantıklar ve kontrolcü komutları simülatördeki aracı ve ortamı yönlendirmektedir.

## Sistem Özellikleri & Konfigürasyon
- **OS:** Linux Ubuntu 24.04
- **ROS Sürümü:** ROS 2 Jazzy
- **Simülatör:** CARLA (`carla_ros_bridge` üzerinden senkron (synchronous) modda haberleşiliyor)
- **Görselleştirme:** Çoğunlukla `foxglove_bridge` üzerinden Foxglove ile veya OpenCV pencereleriyle görselleştirme yapılmaktadır.
- **Harita:** Varsayılan olarak `Town01` kullanılmaktadır. Harita değişikliği `digital_twin.launch.py` içinden yönetilir.

## Araç ve Sensörler (`config/objects.json`)
Ego araç olarak **Tesla Model 3** kullanılmaktadır. Araca eklenen temel sensörler:
- `rgb_front` (Ön RGB Kamera)
- `rgb_view` (Üçüncü şahıs izleme kamerası)
- `depth_front` (Ön Derinlik Kamerası)
- `semantic_front` (Semantik Segmentasyon Kamerası)
- `lidar_top` (Tepede 64 kanallı Lidar)
- `radar_front` (Ön Radar)
- IMU, GNSS, Çarpışma (Collision), Şerit İhlali (Lane Invasion) ve Kilometre Sayacı (Speedometer, Odometry) sensörleri.

## Temel Dosyalar ve Düğümler (Nodes)

### 1. `launch/digital_twin.launch.py` (Ana Başlatıcı)
Simülasyonun çekirdeğini başlatan dosyadır. `ros2 launch digital_twin_pkg digital_twin.launch.py` komutuyla çalışır.
İçerdikleri:
- `carla_ros_bridge`: CARLA ile haberleşmeyi, köprüyü kurar.
- `carla_spawn_objects`: `objects.json` içerisindeki aracı ve sensörleri doğurur.
- `carla_waypoint_publisher` & `local_planner`: Otonom idare, rota oluşturma araçları.
- `robot_state_publisher`: Aracın URDF formatındaki (`urdf/car.urdf`) fiziksel gövdesini yayınlar.
- `foxglove_bridge`: Tüm akışın Foxglove programı üzerinden izlenmesini sağlar.

*(Not: YOLO nesne tanıma, yaya doğurucu ve çukur/enkaz ekleyici sistemler performans, hata ayıklama kolaylığı ve modülerlik açısından bu launch içine dâhil edilmemiş olup, genellikle kendi ayrı terminallerinden `ros2 run` ile başlatılır.)*

### 2. Algılama ve Yapay Zeka Düğümleri
* **`object_detection_node.py`**:
  YOLO11 tabanlı asıl nesne tanıma (object detection) düğümüdür `yolo11_carla_train/weights/best.onnx` ağırlıklarını (fine-tuned model) kullanır. `rgb_front` kamerasından görüntü alır, tahmin yapar. OpenCV penceresi ile anlık görüntüyü basarken, bir yandan da sonuçları `/carla/ego_vehicle/rgb_front/detected_objects` topic'i üzerinden Foxglove'a geri fırlatır.

* **`perception_node.py`**:
  Sensör füzyonu test dosyası. `yolo11n.pt` (standart yolo) kullanarak `rgb_front` ve `depth_front` sensörlerinden gelen verileri eşit zamanlı senkronize eder (`message_filters.ApproximateTimeSynchronizer`). Algılanan yayaların/nesnelerin piksel merkezlerindeki **gerçek derinlik/uzaklık mesafesini (metre cinsinden) hesaplayıp** 15 metreden yakına yaya girdiğinde konsola KRİTİK uyarı basar. CARLA'nın RGB-encoded derinlik formülünü kullanır.

### 3. Simülasyon Ortamını Zenginleştiren Düğümler
* **`spawn_potholes_node.py`**:
  Yol hasarlarını tespit eden YOLO modelini test edebilmek için CARLA haritalarında (Town01 vd.) eksik olan çukurların yerine geçen statik objeleri yollara yapıştırır. `brokentile` (kırık taş) ve `dirtdebris` (toprak/moloz yığını) blueprint'lerini rastgele waypoint'ler üzerinde doğurarak (spawn) asfaltta fiziksel olarak bir yol bozukluğu simülasyonu illüzyonu yaratır. 
    * *Çalıştırma:* `ros2 run digital_twin_pkg spawn_potholes_node`

* **`spawn_pedestrians.py`**:
  Sistemde yaya tespiti yeteneklerini sınamak için CARLA dünyasında gelişmiş AI güdümlü yayalar oluşturan betiktir. CARLA `synchronous` modda olduğu için `rclpy.spin()` yerine kendi custom loop'u ile sürekli `world.wait_for_tick()` fonksiyonunu çağırır. Yayaların %91 oranında yolu karşıdan karşıya geçmesi gibi zorlayıcı senaryo oranları `parameters` olarak mevcuttur.

## Geliştirme Notları (AI Agent İçin Kurallar)
1. **Senkron Mod Zunrulunkları:** CARLA'da senkron (synchronous) mod aktiftir. Dolayısıyla haritaya eklenen AI objelerinin gerçekte hareket edebilmesi için Python üzerinden dünyanın *tick*'lenmesi (`wait_for_tick`) izlenmelidir.
2. **Statik Objeler ve Segmentasyon:** Çukurlar (Potholes) `static.prop.*` sınıfında olduğu için Semantik Segmentasyon kamerasında varsayılan "Static" grubunda (çoğunlukla yeşil tonları) yer alır. Aynı zamanda fiziksel Z-Eksen çarpışma kutularının boyutundan ötürü aracı zıplatabilir veya hafif titretir, bu da bozukluğun gerçekçiliğini artırır.
3. **Prensip - Modüler Test Akışı:** Yeni bir özellik veya deneysel yapay zeka düğümü ekleneceğinde, bunun hemen `digital_twin.launch.py` içine dâhil edilmesi tavsiye EDİLMEZ. Modülerliği korumak ve bağımsız test edebilmek için `setup.py` (entry_points) içine eklenmeli ve konsoldan ayrıca `ros2 run` ile tetiklenmesi hedeflenmelidir. Yalnızca kararlı hale gelen temel / global yapılar launch dosyasında kalmalıdır.
4. **Harita Kısıtlamaları:** Python ile haritaya yerleştirilen object mesh'lerinin (prop) scale(boyut) değeri değiştirilemez, statik prop'ları şeridin sağ-sol ortasına dağıtmak gerekiyorsa waypoint transform lokasyonlarının lokal ekseninde +- uzaklık payı ötelemek matematiksel olarak kolay bir çözümdür. Ayrıca Town07 varsayılan kurulumda gelmeyebilir, `Town01`, `Town03`, `Town10HD` gibi hazır gelenleri tercih et.
