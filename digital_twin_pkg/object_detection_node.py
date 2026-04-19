import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # 1. Load the fine-tuned YOLO11 model
        model_path = '/home/berk/digital_twin_ws/src/digital_twin_pkg/yolo11_carla_train/weights/best.onnx'
        try:
            self.model = YOLO(model_path, task='detect')
            self.get_logger().info(f'YOLO model loaded successfully from: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return

        self.bridge = CvBridge()
        
        # 2. Subscribe to the raw RGB camera frame
        self.rgb_sub = self.create_subscription(
            Image,
            '/carla/ego_vehicle/rgb_front/image',
            self.image_callback,
            10
        )
        
        # 3. Create Publisher for annotated images (To view in Foxglove)
        self.annotated_pub = self.create_publisher(
            Image,
            '/carla/ego_vehicle/rgb_front/detected_objects',
            10
        )
        
        self.get_logger().info('Object Detection Node Started. Waiting for images...')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV Frame
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Predict objects with YOLO (Using half precision to prevent CUDA OOM)
            results = self.model.predict(cv_image, conf=0.4, imgsz=512, half=True, verbose=False)
            
            # Draw bounding boxes (plot gives annotated BGR format image)
            annotated_frame = results[0].plot()
            
            # Also show on screen via OpenCV window
            cv2.imshow("YOLO11 Custom Object Detection", annotated_frame)
            cv2.waitKey(1)
            
            # Convert annotated OpenCV image back to ROS message and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            annotated_msg.header = msg.header # Preserve timestamp and frame_id
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image in callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
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
