import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import message_filters # Senkronizasyon için gerekli

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # 1. Model ve Köprü Kurulumu
        self.model = YOLO('yolo11n.pt') # En hızlı sürüm
        self.bridge = CvBridge()
        
        # 2. Senkronize Abonelikler (RGB ve Depth)
        # İki verinin de aynı anda elimize ulaşması şart
        self.rgb_sub = message_filters.Subscriber(self, Image, '/carla/ego_vehicle/rgb_front/image')
        self.depth_sub = message_filters.Subscriber(self, Image, '/carla/ego_vehicle/depth_front/image')
        
        # Yaklaşık zamanlı senkronizasyon (ApproximateTimeSynchronizer)
        # queue_size=10, slop=0.1 (0.1 saniye fark kabul edilebilir)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.synchronized_callback)
        
        self.get_logger().info('Analiz Katmanı Başlatıldı: Nesne Tespiti + Mesafe Tahmini Aktif.')

    def carla_depth_to_meters(self, pixel_val):
        """
        CARLA'nın RGB-encoded depth verisini metreye çevirir.
        R, G, B değerlerini kullanarak 24-bit derinliği hesaplar.
        """
        # CARLA Depth Formülü: (R + G*256 + B*256*256) / (256^3 - 1) * 1000 metre
        r, g, b = pixel_val
        normalized_depth = (r + g * 256 + b * 256 * 256) / (256**3 - 1)
        return normalized_depth * 1000 # Metre cinsinden mesafe

    def synchronized_callback(self, rgb_msg, depth_msg):
        # 1. Görüntüleri Çevir
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='bgr8')
        
        # 2. YOLO Tahmini
        results = self.model.predict(cv_rgb, conf=0.4, verbose=False)
        
        # 3. Her Tespit İçin Mesafe Hesapla
        for result in results:
            for box in result.boxes:
                # Koordinatları al
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Sınıf ismini al
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                
                # Tespit edilen nesnenin orta noktasındaki derinlik pikselini al
                # Dikkat: OpenCV (y, x) formatında çalışır
                depth_pixel = cv_depth[center_y, center_x]
                distance = self.carla_depth_to_meters(depth_pixel)
                
                # 4. Analiz Loglama ve Görselleştirme
                text = f"{label}: {distance:.2f}m"
                cv2.rectangle(cv_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_rgb, text, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                if label == 'person' and distance < 15.0:
                    self.get_logger().warn(f'KRİTİK: {distance:.2f} metrede yaya tespit edildi!')

        # 5. Sonucu Göster
        cv2.imshow("Digital Twin - Perception & Localization", cv_rgb)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
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