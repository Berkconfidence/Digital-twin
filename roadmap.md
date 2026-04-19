# Yayanın Aracın Kritik Yolunda (Ego-Path) Olup Olmadığını Tespit Etme - Mimari Yol Haritası

Bu belge, otonom sürüş sistemlerinde 2D perspektif yanılsamalarından kaynaklanan hatalı pozitifleri (kaldırımdaki, virajlardaki yayalar vb.) en aza indirmek için yayanın 3D düzlemde aracın rotası üzerinde olup olmadığını saptamaya yönelik aşamalı bir mimari plan sunar. (Hız ve zaman faktörleri hariç tutulmuş, anlık fiziksel konuma odaklanılmıştır.)

---

## Aşama 1: 3D Uzaysal Akıl Yürütme (Pikselden Gerçeğe Geçiş)
Geleneksel 2D bounding box (sınırlayıcı kutu), nesnelerin kamera düzlemindeki genişlik ve yüksekliğini verse de derinlik ve dünya koordinatları (x, y, z) hakkında yetersizdir.

*   **Pikselden Derinliğe İzdüşüm (Sensör Füzyonu):** Sistemimizdeki (`rgb_front`) kameradan alınan YOLO tahminlerini (2D bbox), derinlik kamerasından (`depth_front`) alınan Z-ekseni (mesafe) verileriyle eşleştirerek yayanın alt-orta manipülasyon noktasının (ayaklarının bastığı yerin) kamera merkezine göre 3D (x, y, z) koordinatını hesaplamak.
*   **BEV (Bird's Eye View - Kuş Bakışı Görünüm):** Ters Perspektif Dönüşümü (Inverse Perspective Mapping - IPM) veya kamera matrisleri kullanılarak, yayanın ve aracın konumunun yukarıdan bakılıyormuş gibi aynı 2B harita (X-Y kartezyen düzlemi) üzerinde işaretlenmesi. 
*   **Amaç:** "Yaya benim kameramda kocaman görünüyor, demek ki önümde" yanılgısını yıkıp, "Yaya benden 12.5 metre ileride, 3.2 metre solumda" kesinliğine ulaşmak.

## Aşama 2: Ego-Trajectory ve Kritik Alan (ROI) Modellemesi
Yayanın dünyada nerede olduğunu bulduktan sonra, **bizim** o dünyaya göre nereye doğru hareket ettiğimizi matematiksel olarak tanımlamamız gerekir.

*   **Ego-Path (Araç İlerleme Yolu) Tahmini:** Aracın anlık direksiyon açısı (steering angle) ve dingil mesafesi (wheelbase) kullanılarak, kinematik bisiklet modeli (Kinematic Bicycle Model) ile aracın önümüzdeki birkaç metre boyunca izleyeceği eğrisel rotanın hesaplanması.
*   **Güvenlik Koridoru (Polygon Geometrisi):** Çizilen bu rotanın soluna ve sağına aracın genişliği (Örn: 2m) ve bir güvenlik payı (+0.5m) eklenerek aracın önünde dinamik şekil değiştiren (dönüşlerde kıvrılan) 2 boyutlu bir poligon (koridor) oluşturulur.
*   **Eşleştirme:** 1. Aşamada X-Y düzleminde konumlandırılan yayanın, bu güvenlik koridoru poligonunun içinde (Point-in-Polygon) kalıp kalmadığı kontrol edilir.

## Aşama 3: Yanlış Pozitif Filtreleme (Semantic & Freespace Kontrolü)
Virajlı yollarda güvenlik koridorumuz geometrik olarak doğrudan kaldırıma taşıyor olabilir veya çok geniş bir yol ağzında yaya yolda gibi görünse de güvenli bir refüj üzerinde olabilir.

*   **Sürülebilir Alan (Drivable Area) Çıkarımı:** Araçtaki Semantik Segmentasyon kamerasını (`semantic_front`) kullanarak pikselleri "Yol/Asfalt" ve "Diğerleri (Kaldırım, Araç, Bina vb.)" olarak ikiye ayırmak.
*   **Yere Basma Mantığı:** Yayanın 2D Bounding Box'ının alt orta noktasının (veya 3D uzaydan geri segmantasyon haritasına dönüştürülen noktasının) segmentasyon maskesinde "Sürülebilir Alan" sınıfına (örneğin CARLA'daki yol ID'sine) denk gelip gelmediğini doğrulamak.
*   **Kural:** Yaya koridorumuzun içinde olsa bile, bulunduğu zemin "Yol" değilse (örn. kaldırım piksellerine basıyorsa) frenleme iptal edilir (False Positive Override).

**Projemiz İçin (Digital Twin PKG) "Best Practice" Mimari Kararı:**
Projemizde halihazırda var olan CARLA altyapısı (RGB, Derinlik ve Semantik) düşünüldüğünde, ağır deep learning eğitimlerine girmeden deterministik ve oldukça güçlü çalışacak **Geometrik Füzyon ve Semantik Doğrulama** modelidir.

**Adım Adım Çalışma Mantığı (Implementation Pathway):**
1. YOLO ile `rgb_front` üzerinden yayanın 2D kutusunu bul.
2. Bu kutunun alt-orta (yere basan) kısmındaki pikselin sınıfsal ID'sini `semantic_front` maskesinden oku. Eğer piksel "Kaldırım (Sidewalk/None)" ise yoksay; eğer "Yol (Road/Crosswalk)" ise devam et.
3. Kutu merkezinin derinliğini `depth_front`'tan oku ve Kamera İç Parametreleri (Intrinsics) ile BEV koordinatına (X metre uzaklık, Y metre sağ/sol) çevir.
4. Aracın o anki direksiyon açısı ile ego-path poligonunu hesapla.
5. Yayanın BEV koordinatı bu çokgenin içinde mi diye kontrol et. Cevap evetse -> **Güvenli Fren Komutu Gönder**.
