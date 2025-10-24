import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloDetectorBasic(Node):
    def __init__(self):
        super().__init__('yolo_detector_basic')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8s.pt")

        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.process_image,
            10
        )

        self.detected_objects = {}  # label: confidence
        self.log_timer = self.create_timer(0.5, self.log_summary)

        self.get_logger().info("Basic YOLO detector started")

    def process_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)

        for r in results:
            if r.boxes is not None:
                for box in r.boxes:
                    class_name = self.model.names[int(box.cls)]
                    confidence = float(box.conf)
                    if confidence > 0.5:
                        self.detected_objects[class_name] = confidence

    def log_summary(self):
        if self.detected_objects:
            summary = ", ".join(
                [f"{obj} ({conf:.2f})" for obj, conf in self.detected_objects.items()]
            )
            self.get_logger().info(f"Currently detecting: {summary}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorBasic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
