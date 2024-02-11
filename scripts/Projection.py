#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as Float32Array
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import cv2
import cv_bridge
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


class ProjectionNode(Node):

    def __init__(self):
        # Initialize the ROS node
        super().__init__('Projection')

        self.image_subscriber = self.create_subscription(Image, '/Robot/Astra_rgb/image_color', self.image_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.ray_publisher = self.create_publisher(Float32Array, '/camera_ray', 10)

        self.extrinsic_matrix = None
        intrinsic_path = "/home/esteb37/ocslam/resource/intrinsic_matrix.npy"
        self.intrinsic_matrix = np.load(intrinsic_path)
        distortion_path = "/home/esteb37/ocslam/resource/dist.npy"
        self.distortion = np.load(distortion_path)

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.ax.set_xlim(0, 5)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(0, 0.3)

        # set tiles to 0.5x0.5
        self.ax.set_xticks(np.arange(0, 5, 0.25))
        self.ax.set_yticks(np.arange(-2, 2, 0.25))

        self.points = np.zeros((5,3))
        self.point_index = 0


    def mouse_callback(self, event, x, y, _, __):
        if event == cv2.EVENT_LBUTTONDOWN:
            pixel_coordinate = (x, y)
            self.project_image_point_to_world(pixel_coordinate, self.intrinsic_matrix, self.rotation_matrix, self.translation_vector)



    def image_callback(self, msg):
        image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #ndistorted_image = cv2.undistort(image, self.intrinsic_matrix, self.distortion)

        cv2.namedWindow("image")
        cv2.setMouseCallback("image", self.mouse_callback)

        cv2.imshow("image", image)
        cv2.waitKey(1)


    def pose_callback(self, msg):
        self.rotation_matrix, self.translation_vector = pose_msg_to_numpy(msg.pose.pose)
        #camera is 18 cm above the ground
        self.translation_vector[2] += 0.18


    def project_image_point_to_world(self, image_point, intrinsic_matrix, rotation_matrix, translation_vector):

        focal_length_x = intrinsic_matrix[0, 0]
        focal_length_y = intrinsic_matrix[1, 1]
        principal_point_x = intrinsic_matrix[0, 2]
        principal_point_y = intrinsic_matrix[1, 2]
        x = image_point[0]
        y = image_point[1]
        u = (x - principal_point_x) / focal_length_x
        v = (y - principal_point_y) / focal_length_y

        length = np.sqrt(u**2 + v**2 + 1)
        # the camera is looking towards the positive X plane
        direction_x = 1 / length
        direction_y = -u / length
        direction_z = -v / length

        t = -translation_vector[2] / direction_z
        world_point = translation_vector + t * np.array([direction_x, direction_y, direction_z])



        self.ax.plot([translation_vector[0], world_point[0]], [translation_vector[1], world_point[1]], [translation_vector[2], world_point[2]], c='b')

        # print difference
        self.LOG(expected[self.point_index] - world_point)


        self.point_index += 1


        if self.point_index == 15:
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')

            plt.show()

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