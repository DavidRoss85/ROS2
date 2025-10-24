import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloDetectorViz(Node):
    def __init__(self):
        super().__init__('yolo_detector_viz')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8s.pt")

        self.image_sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.process_image,
            10
        )

        self.viz_pub = self.create_publisher(Image, '/yolo/visualization', 10)
        self.get_logger().info("YOLO visualization node started")

    def process_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, verbose=False)
        annotated = results[0].plot()
        viz_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        self.viz_pub.publish(viz_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
