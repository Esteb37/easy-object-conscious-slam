#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge


class CalibrationNode(Node):

    def __init__(self):
        # Initialize the ROS node
        super().__init__('Calibration')

        self.image_subscriber = self.create_subscription(Image, '/Robot/Astra_rgb/image_color', self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/calibration_image', 10)


    def image_callback(self, msg):
        image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')


        pattern_size = (7,7)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        # If found, add object points, image points
        if ret:
            cv2.drawChessboardCorners(image, pattern_size, corners, ret)

        image_msg = cv_bridge.CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
        self.image_publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    Calibration_node = CalibrationNode()
    rclpy.spin(Calibration_node)
    Calibration_node.destroy_node()


if __name__ == '__main__':
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()