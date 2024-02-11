import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

intrinsic_path = "/home/esteb37/ocslam/resource/intrinsic_matrix.npy"
intrinsic_matrix = np.load(intrinsic_path)

focal_length_x = intrinsic_matrix[0, 0]
focal_length_y = intrinsic_matrix[1, 1]
principal_point_x = intrinsic_matrix[0, 2]
principal_point_y = intrinsic_matrix[1, 2]
image_width = 640
image_height = 480


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


cam_origin = (0, 0, 18)


ax.scatter(cam_origin[0], cam_origin[1], cam_origin[2], c='r', marker='o')

print(focal_length_x, focal_length_y, principal_point_x, principal_point_y)

# Loop through each pixel in the image
for y in range(image_height):
    for x in range(image_width):
        # Calculate normalized device coordinates
        u = (x - principal_point_x) / focal_length_x
        v = (y - principal_point_y) / focal_length_y

        # The ray direction is simply the direction vector (u, v, 1)
        # Normalize the ray direction
        length = np.sqrt(u**2 + v**2 + 1)
        # the camera is looking towards the positive X plane
        direction_x = 1 / length
        direction_y = u / length
        direction_z = v / length


        t = -cam_origin[2] / direction_z
        if x % 100 == 0 and y < principal_point_y and y % 50 == 0:
            world_point = cam_origin + t * np.array([direction_x, direction_y, direction_z])

            # line from the camera to the point
            ax.plot([cam_origin[0], world_point[0]], [cam_origin[1], world_point[1]], [cam_origin[2], world_point[2]], c='b')








ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
