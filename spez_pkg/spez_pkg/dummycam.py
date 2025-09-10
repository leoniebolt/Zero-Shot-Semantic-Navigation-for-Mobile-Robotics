#!/usr/bin/env python3
from rclpy.qos import QoSProfile
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2

class DummyCamera(Node):
    def __init__(self):
        super().__init__('dummy_camera')
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Image, '/camera/image_for_vlfm', qos)
        self.br = CvBridge()
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz

    def publish_frame(self):
        # Einfach ein schwarzes Bild erzeugen
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
