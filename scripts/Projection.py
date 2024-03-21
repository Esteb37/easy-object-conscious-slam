#!/usr/bin/env python3

import rclpy
from Utils import *
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time

class ProjectionNode(LogNode):

    CAM_HEIGHT = 0.18
    MAX_RANGE = 4

    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480

    PLOT = False

    TRACKING_THRESHOLD = 0.1
    ISLAND_THRESHOLD = 0.1

    def __init__(self):
        super().__init__('Projection')

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.yolo_subscriber = self.create_subscription(Int32MultiArray, '/yolo_boxes', self.yolo_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.obj_pub = self.create_publisher(MarkerArray, '/objects', 1)

        self.intrinsic_path = "/home/esteb37/ocslam/resource/intrinsic_matrix.npy"
        self.intrinsic_matrix = np.load(self.intrinsic_path)

        self.rotation_matrix = np.identity(3)
        self.translation_vector = np.array([0, 0, self.CAM_HEIGHT])

        self.camera_projection = Projection([
                                  (0, 0),
                                  (self.IMAGE_WIDTH, 0),
                                  (self.IMAGE_WIDTH, self.IMAGE_HEIGHT),
                                  (0, self.IMAGE_HEIGHT)],
                                  self.intrinsic_matrix,
                                  self.rotation_matrix,
                                  self.translation_vector,
                                  "Camera View",
                                  )


        angles = [np.arctan2(coord[1], coord[0]) for coord in self.camera_projection.array]

        # get angle range
        self.camera_angle_min = min(angles)
        self.camera_angle_max = max(angles)

        if self.PLOT:
          self.fig = plt.figure()
          self.ax = self.fig.add_subplot(121)
          self.ax2 = self.fig.add_subplot(122)

        self.yolo_projections = []

        self.lidar = []
        self.angle_min = 0
        self.angle_increment = 0

        self.objects = []

    def yolo_callback(self, msg):

        self.yolo_projections = []

        if self.PLOT:
          self.ax2.clear()
          self.ax2.set_xlim(0, self.IMAGE_WIDTH)
          self.ax2.set_ylim(0, self.IMAGE_HEIGHT)

        for i in range(0, len(msg.data), 6):
            x1, y1, x2, y2, class_index, confidence = msg.data[i:i+6]
            y1 = self.IMAGE_HEIGHT - y1
            y2 = self.IMAGE_HEIGHT -  y2

            label = CLASS_NAMES[class_index]

            if self.PLOT:
              color = string_to_rgbf(label)
              self.ax2.plot([x1, x2, x2, x1, x1], [y1, y1, y2, y2, y1], color=color, label=label)

            self.yolo_projections.append(Projection(
                                          [(x1, y2), # Top Left
                                          (x2, y2), # Bottom Left
                                          (x2, y1), # Bottom Right
                                          (x1, y1)], # Top Right
                                          self.intrinsic_matrix,
                                          self.rotation_matrix,
                                          self.translation_vector,
                                          label, confidence,
                                        ))

        if self.PLOT:
          self.ax2.legend()

        self.classify_markers()

    def pose_callback(self, msg):
        pass

    def lidar_callback(self, msg):
        self.lidar = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment


    def classify_markers(self):
        # if none or empty return
        if not self.lidar:
            return

        if self.PLOT:
          self.ax.clear()
          self.ax.set_xlim(0, self.MAX_RANGE*1.1)
          self.ax.set_ylim(-self.MAX_RANGE*1.1, self.MAX_RANGE*1.1)

          # Plot camera bound
          self.camera_projection.plot(self.ax, "--","red")

          for projection in self.yolo_projections:
            projection.plot(self.ax, "-")

          self.ax.legend()

        groups = {}

        for point in self.lidar:
            if point != float('inf'):
                # get the angle of the point
                angle = self.angle_min + self.angle_increment * self.lidar.index(point)
                # get the x and y coordinates of the point
                x = point * np.cos(angle)
                y = point * np.sin(angle)

                if angle > self.camera_angle_min and angle < self.camera_angle_max:
                    found = False
                    for projection in self.yolo_projections:
                        if projection.contains(x, y):
                            groups.setdefault(projection.label, []).append((x, y, projection.confidence))

                            if self.PLOT:
                              self.ax.plot(x, y, color=projection.color, marker='.')
                              found = True

                    if self.PLOT and not found:
                      self.ax.plot(x, y, 'r.')

        if not groups:
            return

        for label, points in groups.items():

          new_object = find_largest_island([(x,y) for x, y, _ in points], self.ISLAND_THRESHOLD, label)

          if new_object is not None:
            tracked = False

            for obj in self.objects:
                if new_object.label == obj.label and new_object.within_distance(obj, self.TRACKING_THRESHOLD):
                    obj.update_data(new_object)
                    tracked = True

            if not tracked:
              self.objects.append(new_object)

        markers = MarkerArray()
        marker_id = 0
        for obj in self.objects:
            markers.markers.append(obj.as_marker(marker_id))
            marker_id+=1

            if self.PLOT:
              self.ax.add_patch(obj.as_ellipse())

        if self.PLOT:
          self.ax.legend()
          self.fig.canvas.draw()
          self.fig.canvas.flush_events()
          plt.pause(0.0001)

        self.obj_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    projection_node = ProjectionNode()
    rclpy.spin(projection_node)
    projection_node.destroy_node()


if __name__ == '__main__':
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()