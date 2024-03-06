#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, MultiPoint
from shapely.ops import unary_union
from matplotlib.patches import Ellipse


def pose_msg_to_numpy(pose_msg):
    # Extract rotation quaternion from pose message
    quat = np.array([pose_msg.orientation.x, pose_msg.orientation.y,
                     pose_msg.orientation.z, pose_msg.orientation.w])
    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(quat)
    # Extract translation vector from pose message
    translation_vector = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    return rotation_matrix, translation_vector

def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    rotation_matrix = np.array([[1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                                [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
                                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])
    return rotation_matrix

def string_to_rgb_color(string):
    # Compute the hash value of the string
    hash_value = hash(string)

    # Extract the RGB components from the hash value
    red = (hash_value & 0xFF0000) >> 16
    green = (hash_value & 0x00FF00) >> 8
    blue = hash_value & 0x0000FF

    return (blue / 255, green / 255, red / 255)

def x_vals(array):
  return [point[0] for point in array] + [array[0][0]]

def y_vals(array):
  return [point[1] for point in array] + [array[0][1]]

from shapely.geometry import MultiPolygon

def find_islands(points, threshold):

  # Convert points to Shapely Point objects
  shapely_points = [Point(x, y) for x, y in points]

  # Create MultiPoint from the list of Shapely Point objects
  multi_point = MultiPoint(shapely_points)

  # Buffer the MultiPoint to form circles around each point
  buffered_multi_point = multi_point.buffer(threshold)

  # Merge overlapping circles to form continuous regions
  merged_multi_point = unary_union(buffered_multi_point)

  # Extract the individual polygons representing connected regions
  if isinstance(merged_multi_point, MultiPolygon):
     islands = list(merged_multi_point.geoms)
  else:
    islands =  [merged_multi_point]

  ellipses = []
  for island in islands:
      # Get the minimum bounding box of the island
      min_x, min_y, max_x, max_y = island.bounds
      center_x = (min_x + max_x) / 2
      center_y = (min_y + max_y) / 2
      width = max_x - min_x
      height = max_y - min_y

      # Calculate rotation angle of the ellipse
      # For simplicity, assume the rotation angle is 0
      rotation = 0

      # Append ellipse parameters to the list
      ellipses.append(((center_x, center_y),width,height,rotation))

  return ellipses


class ProjectionNode(Node):

    CLASS_NAMES = [ "ball",
                    "dog",
                    "gnome",
                    "table",
                    "chair",
                    "door",
                    "duck",
                    "person",
                    "extinguisher",
                    "flowers",
                    "shelf"]

    CAM_HEIGHT = 0.18
    MAX_RANGE = 4

    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480

    def __init__(self):
        super().__init__('Projection')

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.yolo_subscriber = self.create_subscription(Int32MultiArray, '/yolo_boxes', self.yolo_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.extrinsic_matrix = None
        intrinsic_path = "/home/esteb37/ocslam/resource/intrinsic_matrix.npy"
        intrinsic_matrix = np.load(intrinsic_path)

        self.focal_length_x = intrinsic_matrix[0, 0]
        self.focal_length_y = intrinsic_matrix[1, 1]
        self.principal_point_x = intrinsic_matrix[0, 2]
        self.principal_point_y = intrinsic_matrix[1, 2]

        self.rotation_matrix = np.identity(3)
        self.translation_vector = np.array([0, 0, self.CAM_HEIGHT])

        self.camera_corners = [self.project_point(corner) for corner in
                        [(0, 0),
                         (self.IMAGE_WIDTH, 0),
                         (self.IMAGE_WIDTH, self.IMAGE_HEIGHT),
                         (0, self.IMAGE_HEIGHT)]]

        angles = [np.arctan2(coord[1], coord[0]) for coord in self.camera_corners]

        # get angle range
        self.camera_angle_min = min(angles)
        self.camera_angle_max = max(angles)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)

        self.yolo_corners = []





    def yolo_callback(self, msg):
        self.yolo_corners = []
        self.ax2.clear()
        self.ax2.set_xlim(0, self.IMAGE_WIDTH)
        self.ax2.set_ylim(0, self.IMAGE_HEIGHT)

        for i in range(0, len(msg.data), 6):
            x1, y1, x2, y2, class_index, confidence = msg.data[i:i+6]
            y1 = self.IMAGE_HEIGHT - y1
            y2 = self.IMAGE_HEIGHT -  y2

            label = self.CLASS_NAMES[class_index]
            color = string_to_rgb_color(label)
            self.ax2.plot([x1, x2, x2, x1, x1], [y1, y1, y2, y2, y1], color=color, label=label)

            top_right = self.project_point([x1, y1])
            bottom_left = self.project_point([x2, y2])
            bottom_right = self.project_point([x2, y1])
            top_left = self.project_point([x1, y2])

            self.yolo_corners.append([top_left, bottom_left, bottom_right, top_right, label, confidence])

        self.ax2.legend()

    def pose_callback(self, msg):
        pass

    def lidar_callback(self, msg):
        self.lidar = msg.ranges

        # if none or empty return
        if not self.lidar:
            return

        self.ax.clear()
        self.ax.set_xlim(0, self.MAX_RANGE*1.1)
        self.ax.set_ylim(-self.MAX_RANGE*1.1, self.MAX_RANGE*1.1)

        # Plot camera bound
        self.ax.plot(x_vals(self.camera_corners),y_vals(self.camera_corners), 'r--', label="Camera view")

        for box in self.yolo_corners:
          corners = box[:4]
          label = box[4]
          color = string_to_rgb_color(label)
          self.ax.plot(x_vals(corners), y_vals(corners), color=color, linestyle="-", label=label)

        self.ax.legend()


        groups = {}

        for point in self.lidar:
            if point != float('inf'):
                # get the angle of the point
                angle = msg.angle_min + msg.angle_increment * self.lidar.index(point)
                # get the x and y coordinates of the point
                x = point * np.cos(angle)
                y = point * np.sin(angle)

                if angle > self.camera_angle_min and angle < self.camera_angle_max:
                    found = False
                    for box in self.yolo_corners:
                        if Polygon(box[:4]).contains(Point(x, y)):
                            label = box[4]
                            self.ax.plot(x, y, color=string_to_rgb_color(label), marker='.')
                            found = True
                            groups.setdefault(label, []).append((x, y, box[5]))

                    if not found:
                      self.ax.plot(x, y, 'r.')

        # create different arrays for each class

        for label, points in groups.items():
          islands = find_islands([(x,y) for x, y, _ in points], 0.1)
          for island in islands:
            center, width, height, rotation = island
            self.ax.add_patch(Ellipse(center, width, height, rotation, color=string_to_rgb_color(label), fill=False, linestyle="--"))

        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.0001)


    def project_point(self, image_point):
        x = image_point[0]
        y = image_point[1]
        u = (x - self.principal_point_x) / self.focal_length_x

        v = (min(y, self.principal_point_y - 1) - self.principal_point_y) / self.focal_length_y
        length = np.sqrt(u**2 + v**2 + 1)
        # the camera is looking towards the positive X plane
        direction_x = 1 / length
        direction_y = -u / length
        direction_z = v / length

        direction = np.dot(self.rotation_matrix, [direction_x, direction_y, direction_z])

        t = -self.translation_vector[2] / direction[2]
        world_point = self.translation_vector + t * np.array([direction[0], direction[1], direction[2]])
        return world_point

    def LOG(self, msg):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    projection_node = ProjectionNode()
    rclpy.spin(projection_node)
    projection_node.destroy_node()


if __name__ == '__main__':
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()