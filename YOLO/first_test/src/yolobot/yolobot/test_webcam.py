# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

DEFAULT_CAMERA = 0

class WebCamDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO("yolov8s.pt")
        self.capture = None
        self.camera_number = DEFAULT_CAMERA
        self.detected_objects = {}

    def open_camera(self):
        # Open the camera directly using the device path
        self.capture = cv2.VideoCapture(self.camera_number)


        # Check if the camera opened successfully
        if not self.capture.isOpened():
            print("Error: Could not open video device.")
            # exit()
            return
        cv2.namedWindow("this",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("this", 1920,1080)
        

    def capture_image(self):
        ret, frame = self.capture.read()
        if ret:
            # print("captured image")
            self.process_image(frame)
            
            # cv2.imshow("this",frame)
        else:
            print("Couldn't grab frame")

    def process_image(self, frame):
        cv_image = frame #self.bridge.imgmsg_to_cv2(frame, 'bgr8')
        results = self.model(cv_image, verbose=False)
        self.detected_objects = {}
        for r in results:
            if r.boxes is not None:
                for box in r.boxes:
                    class_name = self.model.names[int(box.cls)]
                    confidence = float(box.conf)
                    if confidence > 0.5:
                        self.detected_objects[class_name] = confidence
        
        print(self.detected_objects)
        annotated = results[0].plot()
        cv2.imshow("this",annotated)
        cv2.waitKey(1)

    def draw_box_around(self,frame,box,rgb,thickness):
        x1,y1,x2,y2 = map(int, box)
        cv2.rectangle(frame,(x1,y1),(x2,y2),rgb,thickness)


def main(args=None):

    my_cam = WebCamDetector()
    my_cam.open_camera()
    while True:
        my_cam.capture_image()

    pass

if __name__ == '__main__':
    main()
