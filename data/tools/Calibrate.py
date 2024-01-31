import cv2
import numpy as np

def calibrate_camera(images, pattern_size):
    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ..., (8,5,0)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images
    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane


    for image in images:
        copy = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        # If found, add object points, image points
        if ret:
            obj_points.append(objp)
            img_points.append(corners)
            # Draw and display the corners
            cv2.drawChessboardCorners(copy, pattern_size, corners, ret)
            cv2.imshow('image', copy)
            cv2.waitKey(100)
        else:
          print("Corners not found")

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

    return ret, mtx, dist



def pixel_to_ray(pixel_coordinate, intrinsic_matrix):
    # Convert pixel coordinates to normalized device coordinates (NDC)
    ndc = np.linalg.inv(intrinsic_matrix) @ np.append(pixel_coordinate, 1)

    # Convert NDC to ray in camera coordinates (assuming camera is looking along the -z axis)
    ray_camera = np.array([ndc[0], ndc[1], -1])

    return ray_camera / np.linalg.norm(ray_camera)

def ray_intersection(camera_position, ray, surface_normal, surface_point):
    # Calculate t parameter for ray-plane intersection
    t = np.dot(surface_normal, (surface_point - camera_position)) / np.dot(surface_normal, ray)

    # Calculate intersection point in world coordinates
    intersection_point = camera_position + t * ray

    return intersection_point

pixel_coordinate = (0, 0)



# Example calibration images
prefix = "/home/esteb37/ocslam/data/calibration/"
images = [cv2.imread(prefix + str(i) + ".png") for i in range(1, 15)]
pattern_size = (7, 7)  # Size of the calibration pattern (e.g., checkerboard)

# Calibrate the camera
ret, intrinsic_matrix, dist = calibrate_camera(images, pattern_size)

# undistort the images
undistored = [cv2.undistort(image, intrinsic_matrix, dist) for image in images]

# save intrinsic matrix and distortion coefficients
np.save("intrinsic_matrix", intrinsic_matrix)
np.save("dist", dist)

# show the images side by side
for i in range(len(images)):
    cv2.imshow("image", np.hstack((images[i], undistored[i])))
    cv2.waitKey(0)

"""
# Example camera position and orientation in world coordinates
camera_position = np.array([0, 0, 0])  # Camera position in the world (e.g., [x, y, z])

# Example surface parameters (replace with your surface parameters)
surface_normal = np.array([0, 0, 1])  # Normal vector of the surface (e.g., floor plane)
surface_point = np.array([0, 0, 0])   # A point on the surface (e.g., origin)


def mouse_callback(event, x, y, _, __):
    global pixel_coordinate, intrinsic_matrix, camera_position, surface_normal, surface_point
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel_coordinate = (x, y)

        print("Pixel coordinate:", pixel_coordinate)
        print("Intrinsic matrix:", intrinsic_matrix)
        print("Camera position:", camera_position)
        print("Surface normal:", surface_normal)
        print("Surface point:", surface_point)



        ray_camera = pixel_to_ray(pixel_coordinate, intrinsic_matrix)

        print("Ray in camera coordinates:", ray_camera)

        # Calculate intersection point with the surface
        intersection_point = ray_intersection(camera_position, ray_camera, surface_normal, surface_point)
        print("Intersection point in world coordinates:", intersection_point)


# choose a point on the image with mouse
image = images[0]
cv2.namedWindow("image")
cv2.setMouseCallback("image", mouse_callback)
cv2.imshow("image", image)
cv2.waitKey(0)
"""
