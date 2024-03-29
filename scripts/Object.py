
from Utils import *
from shapely.geometry import Point
from visualization_msgs.msg import Marker
from matplotlib.patches import Ellipse
import rclpy


class Object:

  UPDATE_WEIGHT = 0.9
  FRAME_CONFIDENCE = 5


  def __init__(self, center, width, height, projection):
    self.center = center
    self.width = width
    self.height = height
    self.color = projection.color
    self.label = projection.label
    self.presence_confidence = projection.confidence / self.FRAME_CONFIDENCE
    self.detection_confidence = projection.confidence
    self.area = width*height
    self.frame_presence = 1

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

      if self.frame_presence < self.FRAME_CONFIDENCE:
        self.frame_presence += 1

      self.presence_confidence = max(self.presence_confidence, other.detection_confidence * self.frame_presence / self.FRAME_CONFIDENCE)



  def as_marker(self, marker_id, lifetime):
      marker = Marker()
      marker.header.frame_id = "odom"
      marker.type = Marker.CYLINDER
      marker.action = Marker.ADD
      marker.scale.x = self.width
      marker.scale.y = self.height
      marker.scale.z = 0.5
      marker.color.a = self.presence_confidence
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
      text.text = self.label + "," + str(round(self.presence_confidence, 2))+ "%"
      text.id = marker_id * 1000
      text.ns = self.label
      text.lifetime = rclpy.duration.Duration(seconds=lifetime).to_msg()
      return text

  def as_ellipse(self):
    return Ellipse(self.center.xy, self.width, self.height, 0, color=self.color, fill=False, linestyle="--")

  def transform(self, rotation_matrix, translation_vector):
    self.center = Point(np.dot(rotation_matrix, [self.center.x, self.center.y, 0]) + translation_vector)