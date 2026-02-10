import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory
import sys
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


package_path = get_package_share_directory('rpd_percepta')
models_folder = os.path.join(package_path, 'models')


class YoloObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')
        self.get_logger().info('🍃 /yolo_object_detector node has started...')
        # Parameters
        self.declare_parameter('color_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.0)
        color_image_topic = self.get_parameter('color_image_topic').get_parameter_value().string_value
        self.model_name = self.get_parameter('model').get_parameter_value().string_value
        self.conf = self.get_parameter('conf').get_parameter_value().double_value
        # CvBridge and YOLO
        self.bridge = CvBridge()
        self.model = YOLO(os.path.join(models_folder, self.model_name))
        self.executing = False
        # Subscriptions and Publishers
        self.color_image_sub = self.create_subscription(Image, color_image_topic, self.color_image_sub_callback, 1)
        self.detection_plot_pub = self.create_publisher(Image, '/rpd/yolo_detection_plot', 1)
        self.detections_pub = self.create_publisher(Detection2DArray, '/rpd/yolo_detections', 1)

    def color_image_sub_callback(self, msg):
        if self.executing:
            return
        self.executing = True
        self.get_logger().info('📸 Received the image. Performing object detection...')
        result = self.detect_objects(msg)
        self.publish_detection_plot(result)
        self.publish_detections(result, msg.header)
        self.get_logger().info(f'🔍 {len(result.boxes)} objects detected and detections are published!\n')
        self.executing = False

    def detect_objects(self, image):
        image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        cv2.resize(image, (640, 480))
        result = self.model.track(image, conf=self.conf, persist=True)[0]
        return result

    def publish_detection_plot(self, result):
        image = result.plot()
        image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.detection_plot_pub.publish(image)

    def publish_detections(self, result, header):
        detectionArray = Detection2DArray()
        detectionArray.header = header
        for box in result.boxes:
            cls_id = int(box.cls[0])
            cls_name = result.names[cls_id]
            conf = float(box.conf[0])
            cx, cy, w, h = box.xywh[0].tolist()
            detection = Detection2D()
            detection.header = header
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            ohwp = ObjectHypothesisWithPose()
            ohwp.hypothesis.class_id = cls_name
            ohwp.hypothesis.score = conf
            # ohwp.pose = PoseWithCovariance()
            detection.results = [ohwp]
            detection.id = ''
            detectionArray.detections.append(detection)
        self.detections_pub.publish(detectionArray)


def main():
    rclpy.init()
    node = YoloObjectDetector()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().warn(f'Exception occured: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()