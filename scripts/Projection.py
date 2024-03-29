from shapely.geometry import Point, MultiPoint, MultiPolygon, Polygon
from shapely.ops import unary_union
from Utils import *
from Object import *

class Projection:


  ISLAND_THRESHOLD = 0.075

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

  def find_object(self):

    if not self.lidar_points:
      return None

    # Convert points to Shapely Point objects
    shapely_points = [Point(x, y) for x, y in self.lidar_points]

    # Create MultiPoint from the list of Shapely Point objects
    multi_point = MultiPoint(shapely_points)

    # Buffer the MultiPoint to form circles around each point
    buffered_multi_point = multi_point.buffer(self.ISLAND_THRESHOLD)

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
        min_x, min_y, max_x, max_y = island.bounds
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        width = max_x - min_x
        height = max_y - min_y

        dist_to_line = distance_to_line(self.center_line, (center_x, center_y))

        if centermost_island is None or dist_to_line < min_distance:
            centermost_island = Object(Point(center_x, center_y), width, height, self)
            min_distance = dist_to_line

    return centermost_island
