import rclpy
from rclpy.node import Node
import numpy as np

from shapely.ops import unary_union
from visualization_msgs.msg import Marker
from matplotlib.patches import Ellipse
import hashlib
import math

def deterministic_hash(string):
    return hashlib.sha256(string.encode()).hexdigest()

class LogNode(Node):
    def LOG(self, msg):
      self.get_logger().info(str(msg))

def string_to_rgb(string):
    # Compute the hash value of the string
    hash_value = deterministic_hash(string)

    # Extract the RGB components from the hash value
    r = int(hash_value[0:2], 16)
    g = int(hash_value[2:4], 16)
    b = int(hash_value[4:6], 16)

    return [r, g, b]

def string_to_rgbf(string):
  return  [float(val / 255) for val in string_to_rgb(string)]

def quaternion_to_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    rotation_matrix = np.array([[1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                                [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
                                [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])
    return rotation_matrix

def pose_msg_to_numpy(pose_msg):
    # Extract rotation quaternion from pose message
    quat = np.array([pose_msg.orientation.x, pose_msg.orientation.y,
                     pose_msg.orientation.z, pose_msg.orientation.w])
    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(quat)
    # Extract translation vector from pose message
    translation_vector = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    return rotation_matrix, translation_vector

def project_point(image_point, intrinsic_matrix, rotation_matrix, translation_vector):

        focal_length_x = intrinsic_matrix[0, 0]
        focal_length_y = intrinsic_matrix[1, 1]
        principal_point_x = intrinsic_matrix[0, 2]
        principal_point_y = intrinsic_matrix[1, 2]

        x = image_point[0]
        y = image_point[1]
        u = (x - principal_point_x) / focal_length_x

        v = (min(y, principal_point_y - 1) - principal_point_y) / focal_length_y
        length = np.sqrt(u**2 + v**2 + 1)
        # the camera is looking towards the positive X plane
        direction_x = 1 / length
        direction_y = -u / length
        direction_z = v / length

        direction = np.dot(rotation_matrix, [direction_x, direction_y, direction_z])

        t = -translation_vector[2] / direction[2]
        world_point = translation_vector + t * np.array([direction[0], direction[1], direction[2]])
        return world_point

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

def distance_to_line(line, point):
    # Calculate the distance from point to the line defined by start and end
    x1, y1 = line[0]
    x2, y2 = line[1]
    x0, y0 = point
    return abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / math.sqrt((y2-y1)**2 + (x2-x1)**2)
