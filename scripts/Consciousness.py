#!/usr/bin/env python3

from Utils import *
from Object import *
from Projection import *

import rclpy
from std_msgs.msg import Int32MultiArray, Float32
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import time

class Conciousness(LogNode):

    CAM_HEIGHT = 0.20
    LIDAR_HEIGHT = 0.16
    MAX_RANGE = 4

    IMAGE_WIDTH = 640 * 1.05
    IMAGE_HEIGHT = 481 * 1.05

    PLOT = True

    TRACKING_THRESHOLD = 0.1
    PRESENCE_TRESHOLD = 0.0

    def __init__(self):
        super().__init__('Consciousness')

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.yolo_subscriber = self.create_subscription(Int32MultiArray, '/yolo_boxes', self.yolo_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.presence_subscriber = self.create_subscription(Float32, '/presence_threshold', self.presence_callback, 10)

        self.obj_pub = self.create_publisher(MarkerArray, '/objects', 1)

        self.intrinsic_path = "/home/esteb37/ocslam/resource/intrinsic_matrix.npy"
        self.intrinsic_matrix = np.load(self.intrinsic_path)

        self.CAM_LIDAR_DIST = self.CAM_HEIGHT - self.LIDAR_HEIGHT

        self.rotation_matrix = np.identity(3)
        self.translation_vector = np.zeros(3)


        self.camera_projection = Projection([
                                  (0, 0),
                                  (self.IMAGE_WIDTH, 0),
                                  (self.IMAGE_WIDTH, self.IMAGE_HEIGHT),
                                  (0, self.IMAGE_HEIGHT)],
                                  self.intrinsic_matrix,
                                  self.CAM_LIDAR_DIST,
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

        self.time = time.time()

        self.is_new_frame = True

    def presence_callback(self, msg):
        self.PRESENCE_TRESHOLD = msg.data

    def yolo_callback(self, msg):

        self.yolo_projections = []

        if self.PLOT:
          self.ax2.clear()
          self.ax2.set_xlim(0, self.IMAGE_WIDTH)
          self.ax2.set_ylim(0, self.IMAGE_HEIGHT)

        for i in range(0, len(msg.data), 6):
            x1, y1, x2, y2, class_index, confidence = msg.data[i:i+6]
            confidence/=100
            y1 = self.IMAGE_HEIGHT - y1
            y2 = self.IMAGE_HEIGHT -  y2
            x2*=1.05

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
                                          self.CAM_LIDAR_DIST,
                                          label, confidence,
                                        ))

        self.find_objects()
        self.publish_markers()

    def pose_callback(self, msg):
        new_rotation_matrix, new_translation_vector = pose_msg_to_numpy(msg.pose.pose)

        # if the robot has moved or turn, is new frame
        if np.array_equal(new_rotation_matrix, self.rotation_matrix) and np.array_equal(new_translation_vector, self.translation_vector):
            self.is_new_frame = False
        else:
            self.is_new_frame = True

        self.rotation_matrix = new_rotation_matrix
        self.translation_vector = new_translation_vector

    def lidar_callback(self, msg):
        self.lidar = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment


    def find_objects(self):

        if not self.lidar:
            return

        if self.PLOT:
          self.ax.clear()
          self.ax.set_xlim(0, self.MAX_RANGE * 1.1)
          self.ax.set_ylim(-self.MAX_RANGE * 1.1, self.MAX_RANGE * 1.1)

          # Plot camera bound
          self.camera_projection.plot(self.ax, "--","red")

          for projection in self.yolo_projections:
            projection.plot(self.ax, "-")

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
                            projection.add_lidar_point(x, y)

                            if self.PLOT:
                              self.ax.plot(x, y, color=projection.color, marker='.')
                              found = True

                    if self.PLOT and not found:
                      self.ax.plot(x, y, 'r.')

        for projection in self.yolo_projections:

          new_object = projection.find_object()

          if new_object:
            if self.PLOT:
              self.ax.add_patch(new_object.as_ellipse())

            new_object.transform(self.rotation_matrix, self.translation_vector)

            tracked = False

            for obj in self.objects:
                if new_object.label == obj.label and new_object.collides(obj):
                    obj.update_data(new_object, self.is_new_frame)
                    tracked = True

            if not tracked:
              self.objects.append(new_object)


        for i, this in enumerate(self.objects):
            if this is None:
              continue
            for j in range(i+1, len(self.objects)):
                other = self.objects[j]
                if other is None:
                  continue
                if this.collides(other):
                  if this.label == other.label:
                    if this.area > other.area:
                      this.update_data(other, False)
                      self.objects[j] = None
                    else:
                      other.update_data(this, False)
                      self.objects[i] = None
                  else:
                    if this.presence_confidence < other.presence_confidence:
                        self.objects[i] = None
                    else:
                        self.objects[j] = None

        self.objects = [obj for obj in self.objects if obj is not None]

    def publish_markers(self):

        lifetime = (time.time() -  self.time)
        self.time = time.time()

        markers = MarkerArray()
        for marker_id, obj in enumerate(self.objects):
            if obj.presence_confidence < self.PRESENCE_TRESHOLD:
                continue

            markers.markers.append(obj.as_marker(marker_id, lifetime))
            markers.markers.append(obj.as_marker_label(marker_id, lifetime))

        if self.PLOT:
          self.ax.legend()

          if self.yolo_projections:
            self.ax2.legend()
          self.fig.canvas.draw()
          self.fig.canvas.flush_events()
          plt.pause(0.0001)

        # remove objects with same ns and id
        for i, marker in enumerate(markers.markers):
            for j in range(i+1, len(markers.markers)):
                if marker.ns == markers.markers[j].ns and marker.id == markers.markers[j].id:
                    markers.markers[j].id += 1

        self.obj_pub.publish(markers)



def main(args=None):
    rclpy.init(args=args)
    projection_node = Conciousness()
    rclpy.spin(projection_node)
    projection_node.destroy_node()


if __name__ == '__main__':
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()