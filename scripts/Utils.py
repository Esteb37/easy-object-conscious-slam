import rclpy
from rclpy.node import Node
import numpy as np
from shapely.geometry import Point, MultiPoint, MultiPolygon, Polygon
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


class Projection:
  def __init__(self, array,
               intrinsic_matrix,
               height,
               label,
               confidence = 100.0):

    self.array = [project_point(corner,
                                intrinsic_matrix,
                                np.identity(3),
                                np.array([0, 0, height]))
                  for corner in array]

    self.top_left = self.array[0]
    self.bottom_left = self.array[1]
    self.bottom_right = self.array[2]
    self.top_right = self.array[3]


    self.center_line = ((self.top_left[0], (self.top_left[1] + self.bottom_left[1]) / 2),
                        (self.top_right[0], (self.top_right[1] + self.bottom_right[1]) / 2))

    self.label = label
    self.confidence = confidence

    self.color = string_to_rgbf(label)

    self.polygon = Polygon(self.array)

    self.lidar_points = []

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

    ax.plot(*zip(*self.center_line), color= color  if color is not None else self.color, linestyle=":")

  def add_lidar_point(self, x, y):
    self.lidar_points.append((x,y))

  def find_object(self, threshold):

    if not self.lidar_points:
      return None

    # Convert points to Shapely Point objects
    shapely_points = [Point(x, y) for x, y in self.lidar_points]

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

    centermost_island = None
    min_distance = float('inf')
    for island in islands:
        # Get the minimum bounding box of the island
        min_x, min_y, max_x, max_y = island.bounds
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        width = max_x - min_x
        height = max_y - min_y

        dist_to_line = distance_to_line(self.center_line, (center_x, center_y))

        if centermost_island is None:
            centermost_island = Object(self.label, Point(center_x, center_y), width, height)
            min_distance = dist_to_line
        elif dist_to_line < min_distance:
            centermost_island = Object(self.label, Point(center_x, center_y), width, height)
            min_distance = dist_to_line

    return centermost_island

class Object:

  UPDATE_WEIGHT = 0.9

  def __init__(self, label, center, width, height):
    self.label = label
    self.center = center
    self.width = width
    self.height = height
    self.color = string_to_rgbf(self.label)
    self.area = width*height

  def distance(self, other):
    return np.linalg.norm(np.array(self.center.xy) - np.array(other.center.xy))


  def collides(self, other):
    distance = self.distance(other)
    return distance < (self.width + other.width) / 2 or distance < (self.height + other.height) / 2

  def update_data(self, other):
      uw = self.UPDATE_WEIGHT
      uwc = 1 - uw
      self.center = Point(
        (self.center.x  * uw +
         other.center.x * uwc),
        (self.center.y  * uw +
         other.center.y * uwc)
        )

      self.width = max(self.width, other.width)
      self.height = max(self.height, other.height)

  def as_marker(self, marker_id, lifetime):
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
      marker.lifetime = rclpy.duration.Duration(seconds=lifetime).to_msg()
      return marker

  def as_marker_label(self, marker_id, lifetime):
      text = Marker()
      text.header.frame_id = "odom"
      text.type = Marker.TEXT_VIEW_FACING
      text.action = Marker.ADD
      text.scale.z = 0.1
      text.color.a = 1.0
      text.color.r, text.color.g, text.color.b = self.color
      text.pose.position.x = self.center.x
      text.pose.position.y = self.center.y
      text.pose.position.z = 0.5
      text.pose.orientation.w = 1.0
      text.text = self.label
      text.id = marker_id * 1000
      text.ns = self.label
      text.lifetime = rclpy.duration.Duration(seconds=lifetime).to_msg()
      return text

  def as_ellipse(self):
    return Ellipse(self.center.xy, self.width, self.height, 0, color=self.color, fill=False, linestyle="--")

  def transform(self, rotation_matrix, translation_vector):
    self.center = Point(np.dot(rotation_matrix, [self.center.x, self.center.y, 0]) + translation_vector)