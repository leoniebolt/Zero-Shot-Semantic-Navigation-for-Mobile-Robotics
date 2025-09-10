#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Controller(Node):

    def __init__(self):
        super().__init__('ros_controller')
        # Subscriber für Befehle von VLFM
        self.create_subscription(
            Twist,
            '/vlfm/cmd_vel',
            self.cmd_callback,
            10)
        
        # Publisher für TurtleBot4
        self.tb_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

    def cmd_callback(self, msg):
        # Direkt durchreichen an TurtleBot4
        self.get_logger().info(f"Forwarding cmd_vel: {msg.linear.x}, {msg.angular.z}")
        self.tb_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()


#https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
