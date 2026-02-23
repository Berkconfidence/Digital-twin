# Performans ve Bant Genişliği Analiz Raporu

Sistemdeki son güncellemeden (gereksiz kameraların kaldırılması) sonra `digital_twin.launch.py` çalışırken gerçek zamanlı veriler **(top, ros2 topic hz, ros2 topic bw)** toplanmış ve aşağıdaki sonuçlar elde edilmiştir.

## 1. Analiz Sonuçları ve Gözlemlenen İyileştirmeler

Önceki duruma (10+ sensör) göre iletişim performansı kısmında ciddi bir iyileşme gözlemlenmiştir. Ağdaki toplam veri trafiği hafiflemiş, simülasyon frekansında **~%40'lık bir artış** yakalanmıştır.

| Parametre | Optimizasyon Öncesi | Mevcut Durum (Yeni Konfigürasyon) | Durum |
| :--- | :--- | :--- | :--- |
| **Simülasyon Frekansı (Lidar & RGB)** | 4.5 - 5.5 Hz | **6.5 - 7.5 Hz (!)** | **İyileşti** (+ ~2 Hz) |
| **Gereksiz Kamera Bant Genişliği** | ~100 MB/s ekstra yük | **0 MB/s** | **Tasarruf Edildi** |
| **Kalan Kameraların Bant Genişliği** | 25 MB/s (rgb_front) | **~30 MB/s (rgb_front)** <br> **~14 MB/s (rgb_view)*** | **Normal** |

*> Not: `objects.json` dosyasında `rgb_view` kamerası tarafınızca geri eklenmiştir. Bu nedenle 4 kamera üzerinden aktif yayın sürmektedir.*

## 2. Kalan Performans Darboğazları (10 Hz'e Neden Ulaşılamıyor?)

Sistem performansının 7-7.5 Hz bandında takılıp **hedef 10 Hz'e ulaşamamasının** asıl kök nedeni işlemci (CPU) üzerinde gözlemlenen darboğazdır. Terminalde çalışan `top` komutu verilerine göre asıl yük köprü (`bridge`) düğümünde değil, başka bir süreçtedir:

```text
    PID COMMAND               %CPU  %MEM  TIME+  
   6527 CarlaUE4-Linux-      300.0  19.4  6:18.81   -> Carla Server (Normal, GPU/CPU render yüklü)
   6624 carla_manual_control 100.0   1.8  2:00.62   -> DARBOĞAZ (Tek Çekirdek %100 kullanım)
   6621 bridge                41.7   2.0  1:33.56   -> Carla-ROS Bridge (Normal)
```

### Problemlerin Kaynağı:
1. **`carla_manual_control` (Pygame Arayüzü):** `carla_manual_control.py` dosyası genellikle bir **Pygame** (oyun motoru/GUI) penceresi açar ve sensör verilerini kendi formuna render eder. `top` çıktısında işlemcinizin tam 1 çekirdeğini (%100.0) tamamen bu düğümün tükettiği görülüyor.
2. **Serileştirme Yükü (Python GIL):** ROS 2 Python düğümleri (köprü ve manuel kontrol) eşzamanlı çalışmada Python'un GIL kilit mekanizmasına takılır ve tüm sistemi yavaşlatır. Foxglove ile zaten görselleştirme yapıyoruz, bu nedenle Pygame arayüzüne (manuel kontrol arayüzüne) ihtiyacımız yok.

## 3. Çözüm Önerileri (Sonraki Adımlar)

Simülasyon frekansını saf 10 Hz ve üzerine taşıyabilmek için aşağıdaki iyileştirmelerin yapılması gerekmektedir:

1. **`carla_manual_control` Düğümünün Kapatılması:** 
   Eğer Foxglove Studio kullanarak görselleştirme yapıyorsak, `carla_manual_control` pygame ekranından tasarruf edebiliriz. Sürüş komutunu RViz üzerinden veya basit bir ROS 2 `joy_node` (oyun kolu) eklentisi ile sağlayabiliriz. Bu işlemcinize koca bir çekirdek geri kazandıracaktır.
2. **Görüş Kamerasının İnce Ayarı:** 
   Manuel olarak geri eklenen `rgb_view` kamerası saniyede ~15 MB/s yük tüketmektedir. Eğer Foxglove üzerinde sürekli izlenmiyorsa `800x600` yerine `640x480` çözünürlüğe çekilebilir.

Eğer bu önerileri uygulamak isterseniz `launch` dosyasından `manual_control` eklentisini kaldırarak bir sonraki optimizasyon fazına geçebiliriz.
