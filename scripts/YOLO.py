#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import numpy as np
from ultralytics import YOLO

def string_to_rgb_color(string):
  # Compute the hash value of the string
  hash_value = hash(string)

  # Extract the RGB components from the hash value
  red = (hash_value & 0xFF0000) >> 16
  green = (hash_value & 0x00FF00) >> 8
  blue = hash_value & 0x0000FF

  return (blue, green, red)



def msg_to_image(msg):
    width = msg.width
    height = msg.height
    channels = msg.step // msg.width
    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, channels))
    # strip the fourth channel
    if channels == 4:
        img_data = img_data[:, :, :3]
    return img_data

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
    self.model = YOLO("/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/best.pt")
    self.LOG("YOLO Model Loaded")

  def image_to_msg(self, image):
    height, width, channels = image.shape
    encoding = 'bgr8'
    img_msg = Image()
    img_msg.header.stamp = self.get_clock().now().to_msg()
    img_msg.header.frame_id = 'camera_frame'
    img_msg.height = height
    img_msg.width = width
    img_msg.encoding = encoding
    img_msg.step = width * channels
    img_msg.data = image.tobytes()

    return img_msg

  def image_callback(self, msg):
    image = msg_to_image(msg)
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
      cv2.putText(original_image, class_name+" {:.2f}".format(confidences[i]), (int(x1), int(y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


    self.image_publisher.publish(self.image_to_msg(original_image))

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