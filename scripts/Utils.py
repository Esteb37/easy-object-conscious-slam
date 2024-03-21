import rclpy
from rclpy.node import Node
import numpy as np
from shapely.geometry import Point, MultiPoint, MultiPolygon, Polygon
from shapely.ops import unary_union
from visualization_msgs.msg import Marker
from matplotlib.patches import Ellipse

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

def find_largest_island(points, threshold, label):

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

  largest_island = None
  for island in islands:
      # Get the minimum bounding box of the island
      min_x, min_y, max_x, max_y = island.bounds
      center_x = (min_x + max_x) / 2
      center_y = (min_y + max_y) / 2
      width = max_x - min_x
      height = max_y - min_y

      # If the island is the largest one found so far, store it
      if largest_island is None or width * height > largest_island.area:
          largest_island = Object(label, Point(center_x, center_y), width, height)

  return largest_island

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

class Object:
  def __init__(self, label, center, width, height):
    self.label = label
    self.center = center
    self.width = width
    self.height = height
    self.color = string_to_rgbf(self.label)
    self.area = width*height

  def within_distance(self, other, distance):
    return np.linalg.norm(np.array(self.center.xy) - np.array(other.center.xy)) < distance

  def update_data(self, other):
      self.center = Point((self.center.x + other.center.x)/2, (self.center.y + other.center.y)/2)
      self.width = (self.width + other.width)/2
      self.height = (self.height + other.height)/2

  def as_marker(self, marker_id):
      # Publish the object
      marker = Marker()
      marker.header.frame_id = "odom"
      marker.type = Marker.CYLINDER
      marker.action = Marker.ADD
      marker.scale.x = self.width
      marker.scale.y = self.height
      marker.scale.z = 0.5
      marker.color.a = 1.0
      marker.color.r, marker.color.g, marker.color.b = self.color
      marker.pose.position.x = self.center.x
      marker.pose.position.y = self.center.y
      marker.pose.position.z = 0.05
      marker.pose.orientation.w = 1.0
      marker.id = marker_id
      marker.ns = self.label
      marker.lifetime = rclpy.duration.Duration(seconds=0.01).to_msg()
      return marker

  def as_ellipse(self):
    return Ellipse(self.center, self.width, self.height, 0, color=self.color, fill=False, linestyle="--")