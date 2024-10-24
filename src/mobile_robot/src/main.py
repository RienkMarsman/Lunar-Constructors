#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RoverControl(Node):

    def __init__(self):
        super().__init__('rover_control')
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',  # Adjust this topic to match your camera topic
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.last_image = None
        self.print_counter = 0

    def image_callback(self, msg):
        self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def control_loop(self):
        if self.last_image is not None:
            # Simple control: move forward if the center of the image is mostly bright
            height, width, _ = self.last_image.shape
            center_region = self.last_image[int(height/2)-10:int(height/2)+10, int(width/2)-10:int(width/2)+10]
            average_brightness = np.mean(center_region)

            twist = Twist()
            if average_brightness > 100:  # Adjust this threshold as needed
                twist.linear.x = -0.2  # Move forward
            else:
                twist.angular.z = 0.1  # Turn

            self.publisher.publish(twist)

            # Occasionally print camera data
            self.print_counter += 1
            if self.print_counter % 50 == 0:  # Print every 50 iterations (adjust as needed)
                self.get_logger().info(f'Center brightness: {average_brightness}')
                # You could add more image processing and data output here

def main(args=None):
    rclpy.init(args=args)
    rover_control = RoverControl()
    rclpy.spin(rover_control)
    rover_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("hihj")
    main()