import numpy as np

def get_bev_coordinates(pixel_x, distance, fov=90.0, image_width=1280):
    """
    Kamera pikselindeki bir nesneyi 2B Kuş Bakışı (BEV - X, Y) koordinatlarına çevirir.
    
    Parametreler:
    - pixel_x: Nesnenin görüntünün yatay eksenindeki konumu (0-1280 arası)
    - distance: Derinlik kamerasından elde edilen metre cinsinden uzaklık
    - fov: Kameranın yatay görüş açısı (CARLA RGB için 90 varsayılan)
    - image_width: Görüntü genişliği
    
    Dönüş:
    - x_forward: İleriye doğru mesafe (metre)
    - y_lateral: Sağa/Sola doğru yanal sapma (Sağ pozitiftir, Sol negatiftir)
    """
    center_x = image_width / 2.0
    
    # Pikselin merkezden olan sapmasını açıya çevir (derece)
    angle_deg = ((pixel_x - center_x) / image_width) * fov
    angle_rad = np.deg2rad(angle_deg)
    
    # Trigonometrik izdüşüm
    x_forward = float(distance)
    y_lateral = float(distance * np.tan(angle_rad))
    
    return x_forward, y_lateral

def get_ego_path_polygon(steering_angle, max_distance=15.0, car_width=3.0, num_points=10):
    """
    Aracın anlık direksiyon açısına göre önünde oluşturacağı güvenlik koridorunun poligonunu çizer.
    
    Parametreler:
    - steering_angle: -1.0 (tam sol) ile 1.0 (tam sağ) arası yönelim değeri.
    - max_distance: Koridorun metre cinsinden ne kadar uzağa çizileceği.
    - car_width: Koridorun eni. Yayanın araca değme riski olduğu için gerçek araç enine tolerans eklenmiştir (örn 3.0 m).
    
    Dönüş:
    - polygon_points: [(X,Y), (X,Y)...] şeklinde OpenCV uyumlu poligon köşeleri
    """
    # Direksiyonun yörüngeyi ne kadar bükeceği (Parametrik bir parabol sabiti, a)
    # 0.03 çarpanı test için varsayılan ideal değerdir; max_steer'da aracı ~7m sağa saptırır
    curve_factor = 0.03 * steering_angle
    
    left_boundary = []
    right_boundary = []
    
    half_width = car_width / 2.0
    
    # 0'dan max_distance'a kadar X değerleri oluştur (Ego'dan ileriye)
    x_steps = np.linspace(0, max_distance, num_points)
    for x in x_steps:
        # X için merkez Y konumu (Parabol y = a*x^2)
        y_center = curve_factor * (x ** 2)
        
        # Merkez yörüngenin sağında ve solunda sınır noktalarını belirle
        left_boundary.append([x, y_center - half_width])
        right_boundary.append([x, y_center + half_width])
        
    # Sol sınırı ileriye doğru dizip, sağ sınırı geriye doğru dizerek kapalı bir poligon elde edelim
    polygon = left_boundary + right_boundary[::-1]
    
    # OpenCV pointPolygonTest fonksiyonu için float32 numpy tabanlı döndür
    return np.array(polygon, dtype=np.float32)
