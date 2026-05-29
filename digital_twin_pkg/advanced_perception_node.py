import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from carla_msgs.msg import CarlaEgoVehicleStatus  # Direksiyon verisi için
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from ultralytics import YOLO
import message_filters
from digital_twin_pkg.utils.ego_path_math import get_bev_coordinates, get_ego_path_polygon

class AdvancedPerceptionNode(Node):
    def __init__(self):
        super().__init__('advanced_perception_node')
        
        # 1. YOLO Modeli ve CV Bridge Kurulumu
        model_path = '/home/berk/digital_twin_ws/src/digital_twin_pkg/yolo11_carla_train/weights/best.pt'
        try:
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info(f'YOLO model loaded on CPU from: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return
        
        self.bridge = CvBridge()
        
        self.current_steer = 0.0  # Varsayılan düz ilerleme
        
        # 2. Araç Statüsü Aboneliği (Steering vb.)
        self.status_sub = self.create_subscription(
            CarlaEgoVehicleStatus,
            '/carla/ego_vehicle/vehicle_status',
            self.vehicle_status_callback,
            10
        )
        
        # 3. İleri Sensör Füzyonu Abonelikleri (Senkronize)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/carla/ego_vehicle/rgb_front/image')
        self.depth_sub = message_filters.Subscriber(self, Image, '/carla/ego_vehicle/depth_front/image')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.synchronized_callback)
        
        # ═══ Hız ve Fren Kontrol Değişkenleri ═══
        self.declare_parameter('target_speed', 35.0)
        self._brake_active = False
        self._brake_start_time = 0.0
        self._brake_cooldown = 1.0       # saniye

        # ═══ Çukur Yavaşlama Kontrol Değişkenleri ═══
        self._pothole_slowdown_active = False
        self._pothole_last_seen_time = 0.0
        self._pothole_clearance_delay = 1.0  # Çukur geçildikten sonra bekleme süresi (saniye)
        self._pothole_slow_speed = 5      # Çukur tespit edildiğinde hedef hız

        # ═══ Trafik Lambası Kontrol Değişkenleri ═══
        self._red_light_active = False
        self._red_light_last_seen_time = 0.0
        self._red_light_cooldown = 1.0          # Kırmızı ışık kaybolunca bekleme süresi (saniye)
        self._red_light_stop_distance = 5.0     # Bu mesafenin altında dur (metre)
        self._red_light_max_detect_dist = 15.0  # Bundan uzaktaki ışıkları analiz etme (metre)

        # Speed Command Publisher (Artık aracın hızını biz yönetiyoruz)
        speed_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.speed_pub = self.create_publisher(
            Float64,
            '/carla/ego_vehicle/speed_command',
            speed_qos
        )

        self.get_logger().info('Modüler Aşama-2 Katmanı Başlatıldı: Ego-Path ve Yaya Risk Analizi devrede.')

    def _publish_speed(self, speed):
        msg = Float64()
        msg.data = float(speed)
        self.speed_pub.publish(msg)

    def vehicle_status_callback(self, msg):
        # Direksiyon açısını yakalar ve günceller (-1.0 ile 1.0 arası)
        self.current_steer = msg.control.steer

    def draw_bev_map(self, ego_steer, pedestrians_in_path, pedestrians_safe, potholes_in_path, potholes_safe):
        """
        Matematiksel işleyişi test etmek için boş bir siyah ekran üzerinde 
        aracı (BEV), risk koridorunu ve tespit edilen yayaları/çukurları çizer.
        """
        # Siyah zemin oluştur (500x500 px)
        bev_img = np.zeros((500, 500, 3), dtype=np.uint8)
        
        # Harita Ölçeği: 1 metre = 20 piksel (x20 zoom)
        scale = 20
        # Ego Araç Konumu (Ekranın alt orta kısmı - X ileri, Y yana)
        ego_origin_px = (250, 480)  # Px(y_lateral), Px(x_forward tersi)
        
        def to_bev_px(x_m, y_m):
            # X (ileri) ekranın y ekseninde yukarı gitmeli (-)
            # Y (sağ/sol) ekranın x ekseninde hareket etmeli (+)
            px = int(ego_origin_px[0] + (y_m * scale))
            py = int(ego_origin_px[1] - (x_m * scale))
            return (px, py)
        
        # 1. Ego-Path Poligonunu Hesapla ve Çizdir
        ego_polygon = get_ego_path_polygon(ego_steer)
        poly_pts_px = [to_bev_px(pt[0], pt[1]) for pt in ego_polygon]
        poly_pts_px = np.array(poly_pts_px, np.int32).reshape((-1, 1, 2))
        
        # Poligonun içini yarı saydam güvenli koridor rengine (Koyu Kırmızı/Mavi) boya
        corridor_overlay = bev_img.copy()
        cv2.fillPoly(corridor_overlay, [poly_pts_px], (40, 40, 150)) # BGR
        cv2.addWeighted(corridor_overlay, 0.4, bev_img, 0.6, 0, bev_img)
        cv2.polylines(bev_img, [poly_pts_px], isClosed=True, color=(100, 100, 200), thickness=2)
        
        # Ego Aracı (Sarı Nokta veya Dikdörtgen)
        cv2.rectangle(bev_img, (ego_origin_px[0]-10, ego_origin_px[1]), 
                      (ego_origin_px[0]+10, ego_origin_px[1]-30), (0, 255, 255), -1)

        # 2. Yayaları Haritaya Ekle
        for ped in pedestrians_safe:
            px_coord = to_bev_px(ped[0], ped[1])
            cv2.circle(bev_img, px_coord, 5, (0, 255, 0), -1) # Yeşil Nokta: Güvende
            
        for ped in pedestrians_in_path:
            px_coord = to_bev_px(ped[0], ped[1])
            cv2.circle(bev_img, px_coord, 8, (0, 0, 255), -1) # Kırmızı Nokta: Riskli
            cv2.putText(bev_img, "RISK!", (px_coord[0]+10, px_coord[1]-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        # 3. Çukurları Haritaya Ekle (Kare şeklinde)
        for pot in potholes_safe:
            px_coord = to_bev_px(pot[0], pot[1])
            cv2.rectangle(bev_img, (px_coord[0]-4, px_coord[1]-4), (px_coord[0]+4, px_coord[1]+4), (255, 255, 0), -1) # Camgöbeği/Sarı Kare
            
        for pot in potholes_in_path:
            px_coord = to_bev_px(pot[0], pot[1])
            cv2.rectangle(bev_img, (px_coord[0]-5, px_coord[1]-5), (px_coord[0]+5, px_coord[1]+5), (0, 165, 255), -1) # Turuncu Kare
            cv2.putText(bev_img, "CUKUR!", (px_coord[0]+10, px_coord[1]-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,165,255), 2)
            
        cv2.imshow("Aşama-2: BEV & Ego-Path Tracker", bev_img)
        cv2.waitKey(1)

    def synchronized_callback(self, rgb_msg, depth_msg):
        # 1. Görüntüleri OpenCV'e Aktar
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        # CARLA ROS Bridge, derinlik verisini zaten 32FC1 formatında (metre cinsinden float) yayınlar.
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        img_w = cv_rgb.shape[1]
        
        # 2. Poligonu Bellekte Oluştur
        current_polygon = get_ego_path_polygon(self.current_steer)
        
        # 3. Listeler (Harita Çizimi İçin)
        peds_in_path = []
        peds_safe = []
        potholes_in_path = []
        potholes_safe = []
        red_light_detected = False  # Bu frame'de yakın kırmızı ışık var mı?
        
        # 4. YOLO ile Tespit (CPU Inference)
        RELEVANT_LABELS = {'pedestrian', 'pothole', 'trafficLight-Red', 'trafficLight-Green'}
        results = self.model.predict(cv_rgb, conf=0.5, imgsz=512, device='cpu', verbose=False)
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                
                if label not in RELEVANT_LABELS:
                    continue
                    
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = (x1 + x2) // 2
                bottom_y = y2 # Nesnenin alt noktası üzerinden derinlik ölçümü
                
                # Sınır taşmalarını engelle
                bottom_y = min(bottom_y, cv_depth.shape[0] - 1)
                center_x_clamped = min(max(center_x, 0), cv_depth.shape[1] - 1)
                depth_pixel = cv_depth[bottom_y, center_x_clamped]
                
                # Mesafeyi (X) bul (32FC1 formatı doğrudan metredir)
                distance = float(depth_pixel)
                
                # NaN veya uçurum değerlerini (gökyüzü vb.) güvenliğe al
                if np.isnan(distance) or distance <= 0.0 or distance > 1000.0:
                    continue

                # ─── TRAFİK LAMBASI İŞLEME (Ego-path polygon testi YAPILMAZ) ───
                # Trafik lambası yolun dışında kaldırımda yer alır, polygon testi anlamsız.
                # Sadece derinlik kamerasıyla mesafe eşiği kontrolü yapılır.
                if label.startswith('trafficLight'):
                    # Çok uzaktaki ışıkları tamamen yoksay
                    if distance > self._red_light_max_detect_dist:
                        continue
                    
                    if label == 'trafficLight-Red':
                        if distance <= self._red_light_stop_distance:
                            # Yakın kırmızı ışık → DUR
                            red_light_detected = True
                            cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 0, 255), 3)
                            cv2.putText(cv_rgb, f"KIRMIZI ISIK {distance:.1f}m", (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                            self.get_logger().warn(
                                f'\U0001f6a6 KIRMIZI ISIK! Arac durduruluyor. Mesafe: {distance:.1f}m')
                        else:
                            # Uzak kırmızı ışık → bilgi amaçlı etiketle
                            cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 100, 255), 2)
                            cv2.putText(cv_rgb, f"Kirmizi Isik {distance:.1f}m", (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)
                    elif label == 'trafficLight-Green':
                        cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(cv_rgb, f"Yesil Isik {distance:.1f}m", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    continue  # Trafik lambası işlendi, polygon testine gitme

                # ─── YAYA / ÇUKUR İŞLEME (Ego-path polygon testi) ───
                # Çok uzaksa (15 metreden fazla) analize dahil etme
                if distance > 15.0:
                    continue
                    
                # 5. MATEMATİKSEL DÖNÜŞÜM: Pikselden BEV'e (X, Y)
                obj_x, obj_y = get_bev_coordinates(center_x, distance, fov=90.0, image_width=img_w)
                
                # 6. POINT IN POLYGON (Kritik Alan Kontrolü)
                # cv2 pointPolygonTest: Eğrinin içi pozitif, dışı negatif döndürür
                is_inside = cv2.pointPolygonTest(current_polygon, (obj_x, obj_y), measureDist=False)
                
                # 7. Sonucu Ayrıştır ve Ana Ekrana Çiz
                if is_inside >= 0:
                    if label == 'pedestrian':
                        peds_in_path.append((obj_x, obj_y))
                        cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 0, 255), 3) # KIRMIZI
                        cv2.putText(cv_rgb, "Kritik Yaya", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        self.get_logger().warn(f'DİKKAT! Yaya Ego Rota Üzerinde! Uzaklık: {obj_x:.1f}m')
                    elif label == 'pothole':
                        potholes_in_path.append((obj_x, obj_y))
                        cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 165, 255), 3) # TURUNCU
                        cv2.putText(cv_rgb, "Kritik Cukur", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                        self.get_logger().warn(f'DİKKAT! Cukur Ego Rota Üzerinde! Uzaklık: {obj_x:.1f}m')
                else:
                    if label == 'pedestrian':
                        peds_safe.append((obj_x, obj_y))
                        cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2) # YEŞİL
                        cv2.putText(cv_rgb, "Guvenli Yaya", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    elif label == 'pothole':
                        potholes_safe.append((obj_x, obj_y))
                        cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (255, 255, 0), 2) # AÇIK MAVİ/SARI
                        cv2.putText(cv_rgb, "Guvenli Cukur", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Görüntüleri Bas
        self.draw_bev_map(self.current_steer, peds_in_path, peds_safe, potholes_in_path, potholes_safe)
        
        # ═══ HIZ KONTROL KARAR MEKANİZMASI ═══
        now = time.time()
        current_target_speed = self.get_parameter('target_speed').value

        # --- 1. Çukur Durumunu Güncelle ---
        if potholes_in_path:
            if not self._pothole_slowdown_active:
                self.get_logger().warn(
                    f'⚠️ ÇUKUR TESPİT EDİLDİ! Hız {self._pothole_slow_speed:.0f} km/h\'e düşürülüyor.')
            self._pothole_slowdown_active = True
            self._pothole_last_seen_time = now
        else:
            if self._pothole_slowdown_active:
                elapsed_since_pothole = now - self._pothole_last_seen_time
                if elapsed_since_pothole >= self._pothole_clearance_delay:
                    self._pothole_slowdown_active = False
                    self.get_logger().info(
                        f'✅ Çukur geçildi. Hız {current_target_speed:.1f} km/h\'e yükseltiliyor.')

        # --- 2. Kırmızı Işık Durumunu Güncelle ---
        if red_light_detected:
            if not self._red_light_active:
                self.get_logger().warn('\U0001f6a6 KIRMIZI IŞIK! Araç durduruluyor.')
            self._red_light_active = True
            self._red_light_last_seen_time = now
        else:
            if self._red_light_active:
                elapsed_since_red = now - self._red_light_last_seen_time
                if elapsed_since_red >= self._red_light_cooldown:
                    self._red_light_active = False
                    self.get_logger().info('✅ Kırmızı ışık geçildi / Yeşil ışık.')

        # --- 3. Hız Kararını Ver ---
        # Öncelik: Yaya Freni (0) > Kırmızı Işık (0) > Çukur Yavaşlama (5) > Normal Seyir (35)
        if peds_in_path:
            if not self._brake_active:
                self.get_logger().warn('🛑 ANI FREN! Yaya ego yolunda tespit edildi!')
            self._brake_active = True
            self._brake_start_time = now
            self._publish_speed(0.0)
        else:
            if self._brake_active:
                elapsed = now - self._brake_start_time
                if elapsed >= self._brake_cooldown:
                    self._brake_active = False
                    # Fren kalktıktan sonra diğer durumları kontrol et
                    if self._red_light_active:
                        self.get_logger().info(
                            '✅ Yaya freni kaldırıldı. Kırmızı ışık aktif, hız: 0 km/h')
                        self._publish_speed(0.0)
                    elif self._pothole_slowdown_active:
                        self.get_logger().info(
                            f'✅ Fren kaldırıldı. Çukur aktif, hız: {self._pothole_slow_speed:.0f} km/h')
                        self._publish_speed(self._pothole_slow_speed)
                    else:
                        self.get_logger().info(
                            f'✅ Fren kaldırıldı. Hız: {current_target_speed:.1f} km/h')
                        self._publish_speed(current_target_speed)
                else:
                    self._publish_speed(0.0)
            elif self._red_light_active:
                # Yaya yok ama kırmızı ışık var → dur
                self._publish_speed(0.0)
            elif self._pothole_slowdown_active:
                # Yaya yok, kırmızı ışık yok ama çukur var → yavaşla
                self._publish_speed(self._pothole_slow_speed)
            else:
                # Güvenli seyir, sürekli hedef hızı bas
                self._publish_speed(current_target_speed)

        cv2.imshow("Kamera ve Algilama", cv_rgb)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
