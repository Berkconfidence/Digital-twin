# Performans ve Bant Genişliği Analiz Raporu - v2 (manual_control iptali sonrası)

`carla_manual_control` arayüzünün (Pygame) devre dışı bırakılmasının ardından testler 1280x720 RGB kamera çözünürlüğü ve 90 FOV değeri ile tekrar edilmiş, aşağıdaki sonuçlar elde edilmiştir:

## 1. Simülasyon Frekansı ve İletişim Performansı

| Sensör / Topic | İşlem Öncesi Frekans | **Mevcut Frekans (Foxglove Odaklı)** | Durum |
| :--- | :--- | :--- | :--- |
| **RGB Kamera (rgb_front)** | 6.5 - 7.5 Hz | **7.1 - 7.3 Hz** | Stabil (Artış Yok, ancak daha dengeli) |
| **IMU** | ~8 Hz | **8.7 - 8.9 Hz** | **İyileşti** (+ ~1 Hz) |
| **LiDAR** | ~7 Hz | **~3.5 - 7 Hz (Dalgalı)** | Değişken (Sahnede renderlanacak modele göre anlık darbe) |

## 2. Bant Genişliği Verileri
* **RGB Kamera (rgb_front - 1280x720):** Trafik ortalama **25 - 28 MB/s** aralığında seyretmektedir. Her frekansa karşılık gelen RGB paketinin boyutu tam olarak `3.69 MB` civarıdır ve bu beklenen mükemmel bir değerdir.

## 3. Sistem Yükü ve Darboğaz Analizi (Kök Neden Tespiti)
`carla_manual_control` düğümünün silinmesinin ardından işlemci üzerinde o düğüme ayrılmış olan %100'lük (1 tam çekirdek) devasa yük tamamen ortadan kalkmıştır. Ancak sistem frekansı neden doğrudan 10.0 Hz seviyesine çıkmadı?

Terminal komutlarından elde edilen `top` tablosu şöyle bir sonuç gösteriyor:

```text
    PID COMMAND               %CPU  %MEM  TIME+  
   5928 CarlaUE4-Linux-      308.3  20.0   -> Carla Server (GPU/CPU Render Motoru)
   6024 bridge                83.3   1.9   -> Carla-ROS Bridge
```

**Kalan Darboğaz: `carla_ros_bridge` Seri İşlemi**
Pygame arayüzünü sildiğimizde işlemcide yer açıldı ve o çekirdek artık boşta. Fakat `carla_ros_bridge` adlı köprü düğümümüz (python scripti) geri kalan işlemleri kendi başına %83 CPU kullanımıyla yapmaya çalışıyor.

*   Simülasyondan gelen ham görüntü verileri, her "T" anında (simülasyon adımında) köprüye ulaştığında bu python düğümü 1 adet Lidar, 1 adet RGB ve 1 adet Depth/Semantic kamerasını ROS standardı olan paketlere dönüştürerek yayınlamak zorunda.
*   Python'daki **GIL (Global Interpreter Lock)** nedeniyle, bilgisayarınızda ne kadar güçlü çekirdekler olursa olsun, tek bir python dosyasındaki işlemler paralel (gerçek anlamda çok çekirdekli) değil saniyede peş peşe serileştirilmiş biçimde yapılabilir. Bu, 3.69 MB'lık bir verinin kopyalanıp basılma hızını limitliyor.

## 4. Sonuç & Yapılabilecekler

1.  **Mevcut Durum YOLO için Yeterli Mi?**
    *   Bu aşamada 7-8 Hz'lik kesintisiz bir 1280x720 görüntü FPS değeri otonom sürüşteki nesne tespiti modeliniz (YOLO) için yeterli ve tatmin edicidir. Ağ bant genişliğimiz çok ideal sınırlar içinde. Nesne belirginliği mükemmel olacaktır.
2.  **Foxglove Köprüsü (İsteğe Bağlı):**
    *   Yüksek miktar nokta bulutu ve görüntü Foxglove WebSocket ile (foxglove_bridge - saniyede 10.000.000 byte limit konulmuş) gönderiliyor. Eğer ilerleyen günlerde sadece RGB bazlı analiz isterseniz Rviz üzerinden görüntü izlemek, Foxglove köprüsünün yükünü çok az daha hafifletebilir.

**Son Karar:** `manual_control` ekranının verdiği devasa yükü başarıyla kurtardık. Kalan 7-8 hz civarı hızlar şu anki mimari ve donanım limitlerinde ROS 2 köprüsünün bize çıkarabildiği en optimum Python tabanlı dönüşüm hızıdır. Bu sistemle gerçek zamanlı nesne tespiti testlerine geçebilirsiniz.
