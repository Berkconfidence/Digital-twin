import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaEgoVehicleStatus  # Direksiyon verisi için
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import message_filters

# Matematiksel utils modülümüzü import ediyoruz
from digital_twin_pkg.utils.ego_path_math import get_bev_coordinates, get_ego_path_polygon

class AdvancedPerceptionNode(Node):
    def __init__(self):
        super().__init__('advanced_perception_node')
        
        # 1. YOLO Modeli ve CV Bridge Kurulumu
        # CARLA simülatörü GPU VRAM'inin büyük kısmını (~4.5+ GB) tükettiği için,
        # YOLO inference CPU üzerinde çalıştırılıyor. AMD Ryzen 7 6800H (8C/16T)
        # CPU'su, CARLA'nın senkron modundaki 10 tick/s hızıyla uyumlu çalışır.
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
        
        self.get_logger().info('Modüler Aşama-2 Katmanı Başlatıldı: Ego-Path ve Yaya Risk Analizi devrede.')

    def vehicle_status_callback(self, msg):
        # Direksiyon açısını yakalar ve günceller (-1.0 ile 1.0 arası)
        self.current_steer = msg.control.steer

    def draw_bev_map(self, ego_steer, pedestrians_in_path, pedestrians_safe):
        """
        Matematiksel işleyişi test etmek için boş bir siyah ekran üzerinde 
        aracı (BEV), risk koridorunu ve tespit edilen yayaları çizer.
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
        
        # 4. YOLO ile Tespit (CPU Inference)
        # CARLA GPU VRAM'i doldurduğu için YOLO CPU'da çalıştırılıyor.
        # half=True CPU'da desteklenmez, bu yüzden kaldırıldı.
        results = self.model.predict(cv_rgb, conf=0.4, imgsz=512, device='cpu', verbose=False)
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                
                # Sadece Yayalara (person) odaklan
                if label != 'pedestrian':
                    continue
                    
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = (x1 + x2) // 2
                bottom_y = y2 # Yayanın yere bastığı nokta üzerinden derinlik ölçmek daha kesindir
                
                # Sınır taşmalarını engelle
                bottom_y = min(bottom_y, cv_depth.shape[0] - 1)
                depth_pixel = cv_depth[bottom_y, center_x]
                
                # Mesafeyi (X) bul (32FC1 formatı doğrudan metredir)
                distance = float(depth_pixel)
                
                # NaN veya uçurum değerlerini (gökyüzü vb.) güvenliğe al
                if np.isnan(distance) or distance <= 0.0 or distance > 1000.0:
                    continue
                    
                # Çok uzaksa (15 metreden fazla) analize dahil etme
                if distance > 15.0:
                    continue
                    
                # 5. MATEMATİKSEL DÖNÜŞÜM: Pikselden BEV'e (X, Y)
                ped_x, ped_y = get_bev_coordinates(center_x, distance, fov=90.0, image_width=img_w)
                
                # 6. POINT IN POLYGON (Kritik Alan Kontrolü)
                # cv2 pointPolygonTest: Eğrinin içi pozitif, dışı negatif döndürür
                is_inside = cv2.pointPolygonTest(current_polygon, (ped_x, ped_y), measureDist=False)
                
                # 7. Sonucu Ayrıştır ve Ana Ekrana Çiz
                if is_inside >= 0:
                    peds_in_path.append((ped_x, ped_y))
                    cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 0, 255), 3) # KIRMIZI
                    cv2.putText(cv_rgb, "Kritik Yaya", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    self.get_logger().warn(f'DİKKAT! Yaya Ego Rota Üzerinde! Uzaklık: {ped_x:.1f}m')
                else:
                    peds_safe.append((ped_x, ped_y))
                    cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2) # YEŞİL
                    cv2.putText(cv_rgb, "Guvenli", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Görüntüleri Bas
        self.draw_bev_map(self.current_steer, peds_in_path, peds_safe)
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
