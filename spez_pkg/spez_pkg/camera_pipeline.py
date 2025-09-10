#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image   # for recieving image from camera
from cv_bridge import CvBridge
import cv2

class CameraImage(Node):

    def __init__(self):
        super().__init__('camera_image')
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br=CvBridge()

        self.publisher=self.create_publisher(
            Image,
            'camera/image_for_vlfm',
            10
        )

    def listener_callback(self, data):
        #print("In callback")
        #self.get_logger().info('Receiving video frame')
        # current_frame = self.br.imgmsg_to_cv2(data)
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)
        self.publisher.publish(data)    


def main(args=None):
    rclpy.init(args=args)
    camera_image = CameraImage()
    
    try:
        rclpy.spin(camera_image)
    except KeyboardInterrupt:
        pass
    finally:
        camera_image.destroy_node()
        cv2.destroyAllWindwos()
        rclpy.shutdown()

if __name__ == '__main__':
    print("Starting Camera Pipeline.")
    main()


#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://robotics.stackexchange.com/questions/97024/ros2-image-subscriber
#https://ibrahimmansur4.medium.com/integrating-opencv-with-ros2-a-comprehensive-guide-to-computer-vision-in-robotics-66b97fa2de92
