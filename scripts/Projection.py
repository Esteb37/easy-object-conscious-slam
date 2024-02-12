#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

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

        self.corners = [self.project_point(corner) for corner in
                        [(0, 0), (640, 0), (0, 480), (640, 480)]]

        angles = [np.arctan2(coord[1], coord[0]) for coord in self.corners]

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
        self.ax2.set_xlim(0, 640)
        self.ax2.set_ylim(0, 480)
        for i in range(0, len(msg.data), 6):
            x1, y1, x2, y2, confidence, class_index = msg.data[i:i+6]
            y1 = 480 - y1
            y2 = 480 -  y2

            color = string_to_rgb_color(self.CLASS_NAMES[class_index])
            self.ax2.plot([x1, x2, x2, x1, x1], [y1, y1, y2, y2, y1], color=color, label = self.CLASS_NAMES[class_index])
            self.ax2.legend()


            top_left = self.project_point([x1, y1])
            bottom_right = self.project_point([x2, y2])
            top_right = self.project_point([x2, y1])
            bottom_left = self.project_point([x1, y2])
            angles = [np.arctan2(coord[1], coord[0]) for coord in [top_left, top_right, bottom_left, bottom_right]]
            angle_min = min(angles)
            angle_max = max(angles)
            self.yolo_corners.append([top_left, top_right, bottom_left, bottom_right, angle_min, angle_max, class_index])

    def pose_callback(self, msg):
        pass

    def lidar_callback(self, msg):
        self.lidar = msg.ranges

        self.ax.clear()

        self.ax.set_xlim(0, self.MAX_RANGE*1.1)
        self.ax.set_ylim(-self.MAX_RANGE*1.1, self.MAX_RANGE*1.1)

        # plot camera bound
        top_left = self.corners[0]
        top_right = self.corners[1]
        bottom_left = self.corners[2]
        bottom_right = self.corners[3]

        self.ax.plot([top_left[0], top_right[0], bottom_right[0], bottom_left[0], top_left[0]],
               [top_left[1], top_right[1], bottom_right[1], bottom_left[1], top_left[1]], 'r-', label = "Camera view")

        #plot yolo bounds
        for box in self.yolo_corners:
          top_left, top_right, bottom_left, bottom_right, angle_min, angle_max, class_index = box
          color = string_to_rgb_color(self.CLASS_NAMES[class_index])
          self.ax.plot([top_left[0], top_right[0], bottom_right[0], bottom_left[0], top_left[0]],
               [top_left[1], top_right[1], bottom_right[1], bottom_left[1], top_left[1]], color=color, linestyle="--", label = self.CLASS_NAMES[class_index])

        self.ax.legend()

        for point in self.lidar:
            if point != float('inf'):
                # get the angle of the point
                angle = msg.angle_min + msg.angle_increment * self.lidar.index(point)


                # get the x and y coordinates of the point
                x = point * np.cos(angle)
                y = point * np.sin(angle)

                if angle < self.camera_angle_min or angle > self.camera_angle_max:
                    self.ax.plot(x, y, 'b.')
                else:
                    self.ax.plot(x, y, 'r.')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
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

        # clip to 10 meters
        #world_point = np.clip(world_point, -self.MAX_RANGE, self.MAX_RANGE)
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