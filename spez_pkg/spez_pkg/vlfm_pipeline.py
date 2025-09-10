#!/usr/bin/env python3
from rclpy.qos import QoSProfile
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image   # for recieving image from camera
from geometry_msgs.msg import Twist
import math
import time

class VLFMNode(Node):
    def __init__(self):
        super().__init__('vlfm_dummy')
        qos = QoSProfile(depth=10)

        self.cmd_sub = self.create_subscription(
            Image, 
            'camera/image_for_vlfm',
            self.image_callback,
            qos
            )
        
        self.cmd_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            qos
            )
        

        # Liste von Befehlen, die nacheinander ausgeführt werden sollen
        self.commands = [
            "MOVE_FORWARD",
            "TURN_LEFT",
            "MOVE_FORWARD",
            "TURN_RIGHT",
            "MOVE_FORWARD",
            "MOVE_FORWARD",
            "STOP",
            "TURN_LEFT",
            "MOVE_FORWARD"
        ]
        self.current_index = 0
        self.active_timer = None

        # Starte mit dem ersten Befehl
        self.execute_next_command()

        self.last_log_time = 0

    def image_callback(self, msg: Image):
        now = time.time()
        if now - self.last_log_time > 2.0:
            self.get_logger().info("Received an image ✅")
            self.last_log_time = now

    def execute_next_command(self):
        if self.current_index >= len(self.commands):
            self.get_logger().info("Alle Befehle ausgeführt ✅")
            return

        command = self.commands[self.current_index]
        self.execute_vlfm_command(command)
        self.current_index += 1

    def execute_vlfm_command(self, command: str):
        """Führt einen VLFM-Befehl aus und stoppt nach gegebener Zeit"""
        cmd = Twist()
        duration = 0.0

        if command == "MOVE_FORWARD":
            distance = 0.25
            speed = 0.1
            cmd.linear.x = speed
            duration = distance / speed
        elif command == "TURN_LEFT":
            angle = math.radians(30)
            angular_speed = 0.26
            cmd.angular.z = angular_speed
            duration = angle / angular_speed
        elif command == "TURN_RIGHT":
            angle = math.radians(30)
            angular_speed = 0.26
            cmd.angular.z = -angular_speed
            duration = angle / angular_speed
        elif command == "STOP":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            duration = 1.0  # kurze Pause
        else:
            self.get_logger().warn(f"Unknown command: {command}")
            self.execute_next_command()
            return

        # Publish sofort
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"Executing {command} for {duration:.2f} seconds")

        # Nach Ablauf stoppen und nächsten Befehl starten
        if self.active_timer is not None:
            self.active_timer.cancel()
        self.active_timer = self.create_timer(duration, self.stop_and_next)

    def stop_and_next(self):
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info("Stopping robot after action")

        if self.active_timer is not None:
            self.active_timer.cancel()
            self.active_timer = None

        # nächster Befehl
        self.execute_next_command()


def main(args=None):
    rclpy.init(args=args)
    node = VLFMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
