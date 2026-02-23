# Dijital İkiz Sensör Veri ve Performans Analizi

**Döküman Kimliği:** `sensor_data.md`  
**Analiz Tarihi:** 28 Aralık 2025  
**Kapsam:** Gerçek Zamanlı Sensör Telemetrisi ve Sistem Performans Değerlendirmesi

Bu rapor, Tesla Model 3 dijital ikiz projesinin "Bulgular ve Tartışma" bölümü için hazırlanmış teknik analizleri içerir. Veriler, senkron modda çalışan CARLA 0.9.16 simülasyonu ve ROS 2 Jazzy köprüsü üzerinden toplanmıştır.

---

## 1. İletişim Performansı ve Gecikme (Latency) Analizi

Sistemin gerçek zamanlı tepki süresi ve sinyal kararlılığı, temel sensörler (LiDAR, Kamera, IMU) üzerinden incelenmiştir.

### 1.1 Frekans Analizi ve Senkronizasyon Kaybı

| Sensör Tipi | Hedef Frekans (Teorik) | Gözlemlenen Frekans (Ortalama) | Standart Sapma (Jitter) | Sapma Oranı |
| :--- | :--- | :--- | :--- | :--- |
| **LiDAR (Ray-Cast)** | 10.0 Hz | **4.6 - 5.5 Hz** | ±0.15s (Yüksek) | %45 - %54 Düşüş |
| **RGB Kamera** | 10.0 Hz | **4.5 - 5.4 Hz** | ±0.07s (Orta) | %46 - %55 Düşüş |
| **IMU (6-DOF)** | 100.0 Hz | **~5.5 Hz** | ±0.01s (Düşük) | **%94.5 Düşüş** |

### 1.2 Analiz ve Yorumlar

#### 1.2.1 Senkron Mod ve IMU Anomalisi
Gözlemlenen verilerde IMU sensörünün hedef frekansı olan **100Hz yerine sistemin fizik döngüsü olan ~5.5Hz hızında** veri bastığı tespit edilmiştir.
*   **Teorik Neden:** CARLA ROS Bridge, `synchronous_mode: true` parametresi ile çalıştırıldığında, simülasyon sunucusu her bir `tick` (fizik adımı) tamamlanmadan bir sonraki adıma geçmez.
*   **Fizik Adımı:** Simülasyon `fixed_delta_seconds: 0.1` olarak ayarlanmıştır. Bu, teorik tavanı **10 FPS (10 Hz)** ile sınırlar.
*   **Sonuç:** IMU gibi yüksek frekanslı sensörler, simülasyonun "Wall Clock" zamanından bağımsız olarak, sadece simülasyonun hesaplanan her bir karesi için bir (1) veri paketi üretir. Bu durum, 100Hz hedefine ulaşılmasını mimari düzeyde engeller.

#### 1.2.2 Jitter ve Kararlılık
LiDAR ve Kamera verilerinde gözlemlenen **4.6 - 5.5 Hz** aralığındaki dalgalanma (Jitter), sistemin hesaplama darboğazında olduğunu gösterir. Veri akışı sabit bir periyotta değil, işlemcinin o anki yüküne göre değişken aralıklarla gerçekleşmektedir. Bu durum, gerçek zamanlı kontrol algoritmalarında (örn. PID) integral hatasına (windup) yol açabilir.

---

## 2. Bant Genişliği ve Veri Trafiği (Bandwidth)

Ağ üzerindeki veri yükü, özellikle görüntü aktarımı sırasında sistem performansını doğrudan etkileyen bir faktördür.

### 2.1 RGB Kamera Bant Genişliği Modellemesi

Görüntü aktarımı için teorik bant genişliği ihtiyacı aşağıdaki formülle hesaplanmıştır:

$$
Bandwidth = Width \times Height \times Channels \times BitDepth \times Frequency
$$

**Ölçülen Değerler ile Doğrulama:**
*   **Çözünürlük:** $1280 \times 720$ piksel
*   **Kanal Sayısı:** 4 (BGRA - CARLA Ham Çıktısı)
*   **Bit Derinliği:** 8 bit (1 Byte)
*   **Gözlemlenen Frekans:** ~5 Hz (Ortalama)

$$
BW_{teorik} = 1280 \times 720 \times 4 \times 1 \text{ Byte} \times 5 \text{ Hz} \approx 18,432,000 \text{ Byte/s} \approx \textbf{17.58 MB/s}
$$

**Gözlemlenen Veri:** **20 - 27 MB/s**
*   **Sapma Analizi:** Gözlemlenen değerin teorik değerden yüksek çıkması, ROS 2 middleware katmanındaki (DDS) **Header (Zaman damgası, Frame ID)** bilgileri ve paketleme (serialization) maliyetinden kaynaklanmaktadır. Yaklaşık %15-%30 oranında bir "Overhead" tespiti yapılmıştır.

### 2.2 LiDAR Veri Yükü
*   **Gözlemlenen:** 2.4 - 4.4 MB/s.
*   Bu değer, nokta bulutundaki nokta sayısına (çevre engellerin yoğunluğuna) göre dinamik olarak değişmektedir. Şehir içi sahnelerde veri yükü artış göstermektedir.

---

## 3. Hesaplama Kaynakları ve Bottleneck Analizi

Sistemin "End-to-End" performansını kısıtlayan en büyük darboğaz (bottleneck) işlemci tarafında tespit edilmiştir.

### 3.1 Python GIL (Global Interpreter Lock) Çıkmazı
Sistem izleme verileri (htop/top), `carla_ros_bridge` sürecinin tek bir CPU çekirdeğini **%100 (veya ~%80)** kapasiteyle kullandığını göstermektedir.

*   **Teknik Sorun:** `carla_ros_bridge` Python tabanlı bir yazılımdır. Python'un bellek yönetimindeki **GIL (Global Interpreter Lock)** mekanizması, çok çekirdekli bir işlemci üzerinde çalışsa dahi, aynı anda sadece bir iş parçacığının (thread) Python bayt kodunu yürütmesine izin verir.
*   **Etkisi:** Yüksek hacimli veri (örneğin 20MB/s kamera görüntüsü) simülasyondan ROS mesajına dönüştürülürken (Serialization), işlemci tek çekirdekte darboğaza girmekte ve diğer sensörlerin (Lidar, IMU) işlenmesini geciktirmektedir.
*   **Sonuç:** Simülasyon 10 FPS üretmeye çalışsa da, köprü yazılımı veriyi bu hızda serileştirip ROS ağına basamadığı için efektif frekans **~5 Hz** seviyesine düşmektedir.

### 3.2 Tavsiyeler ve Çözüm Önerileri
1.  **C++ Bridge Kullanımı:** Python tabanlı köprü yerine, GIL sınırlaması olmayan C++ tabanlı bir ROS köprüsünün geliştirilmesi performansı 2-3 kat artırabilir.
2.  **Sensör Seyreltme:** Kamera çözünürlüğünün düşürülmesi veya yayın frekansının azaltılması, darboğazı hafifleterek diğer sensörlerin (Lidar) veri akışını rahatlatabilir.
3.  **İş Yükü Dağıtımı:** Kamera ve Lidar gibi ağır sensörlerin farklı ROS düğümleri (nodes) üzerinden, farklı süreçler (multiprocess) olarak çalıştırılması GIL etkisini azaltabilir.
