import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray
from geometry_msgs.msg import Point
import sys

class YoloConvertTo3D(Node):
    def __init__(self):
        super().__init__('yolo_convert_to_3d')
        self.get_logger().info('🍃 /yolo_convert_to_3d node has started...')
        # Parameters
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_image_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.depth_image_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        # CvBridge
        self.bridge = CvBridge()
        self.max_depth = 1000
        # Subsctiptions
        self.get_logger().info('Subscribing to topics...')
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 1)
        self.create_subscription(Image, self.depth_image_topic, self.depth_callback, 1)
        self.create_subscription(Detection2DArray, '/rpd/yolo_detections', self.detections2D_callback, 1)
        self.camera_info = None
        self.depth_image = None
        self.detection_3d_pub = self.create_publisher(Detection3DArray, '/rpd/yolo_detections_3d', 1)

    def info_callback(self, msg):
        self.destroy_subscription(self.camera_info_sub)
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info('📸 Camera info received!')

    def depth_callback(self, msg):
        self.depth_image = msg

    def detections2D_callback(self, msg):
        if self.camera_info is None or self.depth_image is None:
            return
        depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, 'passthrough')
        h, w = depth_image.shape
        detection3DArray = Detection3DArray(header=msg.header)
        for i in msg.detections:
            center = i.bbox.center.position
            x, y = center.x, center.y
            if x >= w:
                x = w - 1
            elif x <= 0:
                x = 0
            if y >= h:
                y = h - 1
            elif y <= 0:
                y = 0
            z = depth_image[int(y), int(x)] / self.max_depth
            x = (x - self.cx) * z / self.fx
            y = (y - self.cy) * z / self.fy
            detection3D = Detection3D(header=i.header, results=i.results, id=i.id)
            detection3D.bbox.center.position = Point(x=float(-y), y=float(x), z=float(z))
            detection3DArray.detections.append(detection3D)
        self.detection_3d_pub.publish(detection3DArray)
        self.get_logger().info(f'🧊 {len(detection3DArray.detections)} 3D detections are published!\n')
        self.depth_image = None


def main():
    rclpy.init()
    node = YoloConvertTo3D()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().warn(f'Exception occured : {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
