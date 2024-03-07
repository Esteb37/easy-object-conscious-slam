from rclpy.node import Node
import numpy as np
from shapely.geometry import Point, MultiPoint, MultiPolygon, Polygon
from shapely.ops import unary_union

class LogNode(Node):
    def LOG(self, msg):
      self.get_logger().info(str(msg))

def string_to_rgb(string):
    # Compute the hash value of the string
    hash_value = hash(string)

    # Extract the RGB components from the hash value
    red = (hash_value & 0xFF0000) >> 16
    green = (hash_value & 0x00FF00) >> 8
    blue = hash_value & 0x0000FF

    return (blue, green, red)

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

class Projection:
  def __init__(self, array,
               intrinsic_matrix,
               rotation_matrix,
               translation_vector,
               label, confidence = 100.0):

    self.array = [project_point(corner,
                                    intrinsic_matrix,
                                    rotation_matrix,
                                    translation_vector)
                  for corner in array]

    self.top_left = self.array[0]
    self.bottom_left = self.array[1]
    self.bottom_right = self.array[2]
    self.top_right = self.array[3]

    self.label = label
    self.confidence = confidence

    self.color = string_to_rgbf(label)

    self.polygon = Polygon(self.array)

  def x_perim(self):
    return [point[0] for point in self.array] + [self.array[0][0]]

  def y_perim(self):
    return [point[1] for point in self.array] + [self.array[0][1]]

  def contains(self, x, y):
    return self.polygon.contains(Point(x, y))

  def plot(self, ax, linestyle, color = None):
    ax.plot(self.x_perim(),
            self.y_perim(),
            color= color  if color is not None else self.color,
            linestyle=linestyle,
            label=self.label)
