#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
import cv2
import cv_bridge
import numpy as np
import matplotlib.pyplot as plt

def string_to_rgb_color(string):
    # Compute the hash value of the string with seed 0
    hash_value = hash(string)

    # Extract the RGB components from the hash value
    red = (hash_value & 0xFF0000) >> 16
    green = (hash_value & 0x00FF00) >> 8
    blue = hash_value & 0x0000FF

    return (blue / 255.0, green / 255.0, red / 255.0)


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

    def __init__(self):
        # Initialize the ROS node
        super().__init__('Projection')

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.yolo_subscriber = self.create_subscription(Int32MultiArray, '/yolo_boxes', self.yolo_callback, 10)

        self.extrinsic_matrix = None
        intrinsic_path = "/home/esteb37/ocslam/resource/intrinsic_matrix.npy"
        intrinsic_matrix = np.load(intrinsic_path)

        self.focal_length_x = intrinsic_matrix[0, 0]
        self.focal_length_y = intrinsic_matrix[1, 1]
        self.principal_point_x = intrinsic_matrix[0, 2]
        self.principal_point_y = intrinsic_matrix[1, 2]

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(121, projection='3d')
        self.ax.set_zlim(0, 0.20)



        self.ax2 = self.fig.add_subplot(122)




    def yolo_callback(self, msg):
        self.ax.clear()
        self.ax2.clear()

        self.ax2.set_xlim(0, 640)
        self.ax2.set_ylim(0, 480)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')

        camera_origin = self.translation_vector

        self.ax.scatter(camera_origin[0], camera_origin[1], camera_origin[2], c='r', marker='o')

        for i in range(0, len(msg.data), 6):
            x1, y1, x2, y2, confidence, class_index = msg.data[i:i+6]

            top_left = self.project_image_point_to_world([x1, y1], self.rotation_matrix, self.translation_vector)
            bottom_right = self.project_image_point_to_world([x2, y2], self.rotation_matrix, self.translation_vector)
            top_right = self.project_image_point_to_world([x2, y1], self.rotation_matrix, self.translation_vector)
            bottom_left = self.project_image_point_to_world([x1, y2], self.rotation_matrix, self.translation_vector)

            color = (string_to_rgb_color(self.CLASS_NAMES[class_index]))

            self.ax.plot([top_right[0], bottom_left[0]], [top_right[1], bottom_left[1]], [top_right[2], bottom_left[2]], color=color)
            self.ax.plot([top_left[0], bottom_right[0]], [top_left[1], bottom_right[1]], [top_left[2], bottom_right[2]], color=color)
            self.ax.plot([top_right[0], top_left[0]], [top_right[1], top_left[1]], [top_right[2], top_left[2]], color=color)
            self.ax.plot([bottom_right[0], bottom_left[0]], [bottom_right[1], bottom_left[1]], [bottom_right[2], bottom_left[2]], color=color)

            self.ax.plot([top_right[0], camera_origin[0]], [top_right[1], camera_origin[1]], [top_right[2], camera_origin[2]], color=color,  linestyle='dashed')
            self.ax.plot([bottom_left[0], camera_origin[0]], [bottom_left[1], camera_origin[1]], [bottom_left[2], camera_origin[2]], color=color,  linestyle='dashed')
            self.ax.plot([top_left[0], camera_origin[0]], [top_left[1], camera_origin[1]], [top_left[2], camera_origin[2]], color=color, linestyle='dashed')
            self.ax.plot([bottom_right[0], camera_origin[0]], [bottom_right[1], camera_origin[1]], [bottom_right[2], camera_origin[2]], color=color, linestyle='dashed')

            # invert y
            y1 = 480 - y1
            y2 = 480 - y2

            self.ax2.plot([x1, x2], [y1, y1], color=color)
            self.ax2.plot([x2, x2], [y1, y2], color=color)
            self.ax2.plot([x2, x1], [y2, y2], color=color)
            self.ax2.plot([x1, x1], [y2, y1], color=color)
            self.ax2.text(x1, y1, self.CLASS_NAMES[class_index], color=color)





        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.0001)


    def pose_callback(self, msg):
        self.rotation_matrix, self.translation_vector = pose_msg_to_numpy(msg.pose.pose)
        #camera is 18 cm above the ground
        self.translation_vector[2] += 0.18


    def project_image_point_to_world(self, image_point, rotation_matrix, translation_vector):
        x = image_point[0]
        y = image_point[1]
        u = (x - self.principal_point_x) / self.focal_length_x

        v = (min(y, self.principal_point_y - 1) - self.principal_point_y) / self.focal_length_y
        length = np.sqrt(u**2 + v**2 + 1)
        # the camera is looking towards the positive X plane
        direction_x = 1 / length
        direction_y = -u / length
        direction_z = -v / length

        direction = np.dot(rotation_matrix, [direction_x, direction_y, direction_z])

        t = -translation_vector[2] / direction[2]
        world_point = translation_vector + t * np.array([direction[0], direction[1], direction[2]])

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