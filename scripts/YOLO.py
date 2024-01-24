#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from ultralytics import YOLO
import cv_bridge

def string_to_rgb_color(string):
    # Compute the hash value of the string
    hash_value = hash(string)

    # Extract the RGB components from the hash value
    red = (hash_value & 0xFF0000) >> 16
    green = (hash_value & 0x00FF00) >> 8
    blue = hash_value & 0x0000FF

    return (blue, green, red)

class YOLONode(Node):

    WIDTH = 224
    HEIGHT = 168
    CONFIDENCE_THRESHOLD = 0.7

    def __init__(self):
        # Initialize the ROS node
        super().__init__('YOLO')

        self.image_subscriber = self.create_subscription(Image, '/Robot/Astra_rgb/image_color', self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/yolo_image', 10)

        self.LOG("Loading YOLO Model")
        self.model = YOLO("/home/esteb37/ocslam/resource/best.pt")
        self.LOG("YOLO Model Loaded")



    def image_callback(self, msg):
        image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        original_image = image.copy()
        image = cv2.resize(image, (self.WIDTH, self.HEIGHT))
        results = self.model(image, imgsz=(self.HEIGHT, self.WIDTH), conf = self.CONFIDENCE_THRESHOLD)
        boxes = results[0].boxes
        ratio = original_image.shape[0] / self.HEIGHT
        positions = boxes.xyxy.cpu().numpy() * ratio
        confidences = boxes.conf.cpu().numpy()
        classes = boxes.cls.cpu().numpy()
        class_names = results[0].names
        for i, box in enumerate(positions):
            x1, y1, x2, y2 = box
            class_name = class_names[classes[i]]
            color = string_to_rgb_color(class_name)
            cv2.rectangle(original_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)


            message = class_name+" {:.2f}".format(confidences[i])
            (text_width, text_height) = cv2.getTextSize(message, cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4, thickness=1)[0]
            cv2.rectangle(original_image, (int(x1), int(y1)), (int(x1 + text_width), int(y1 - text_height)), color, cv2.FILLED)
            foreground = (0, 0, 0) if color[0] + color[1] + color[2] > 255 else (255, 255, 255)

            cv2.putText(original_image, message, (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4, color=foreground, thickness=1)

        image_msg = cv_bridge.CvBridge().cv2_to_imgmsg(original_image, encoding='bgr8')
        self.image_publisher.publish(image_msg)

    def LOG(self, msg):
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()


if __name__ == '__main__':
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()