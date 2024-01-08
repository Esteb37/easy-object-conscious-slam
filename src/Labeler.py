import os
import cv2
import numpy as np
import time

clas = "Table" # "2"

# cambiar para tu path
origin_path = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/images/"+clas

# igual cambiar
output_path = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/outputs"



if not os.path.exists(output_path):
    os.makedirs(output_path)

# no cambiar nada aquí



top_right = None
bottom_left = None
temp_bottom_left = None

def drag_event(event, x, y, flags, param):
	global top_right, bottom_left, temp_bottom_left
	if event == cv2.EVENT_LBUTTONDOWN:
		top_right = (x, y)
	# while draggin
	elif event == cv2.EVENT_MOUSEMOVE and top_right is not None:
		temp_bottom_left = (x, y)
	# when draggin stops
	elif event == cv2.EVENT_LBUTTONUP:
		bottom_left = (x, y)

cv2.namedWindow("image")
cv2.setMouseCallback("image", drag_event)

i = 0
while i < len(os.listdir(origin_path)):

    file = os.listdir(origin_path)[i]
    name = origin_path + "/" + file
    # check if there is a label already

    if os.path.isfile( output_path +"/"+ file[:-4] + ".txt"):
        continue

    img = cv2.imread(name)
    cont = True
    if img is not None:
        while top_right is None or bottom_left is None:
            copy = img.copy()
            if (top_right is not None and temp_bottom_left is not None):
                cv2.rectangle(copy, top_right, temp_bottom_left, (0, 255, 0), 2)
            cv2.imshow("image", copy)
            if cv2.waitKey(1) == 13:
                cont = False
                break

    if not cont:
        i+=1
        continue
    center_x = int((top_right[0] + bottom_left[0]) / 2) / 224
    center_y = int((top_right[1] + bottom_left[1]) / 2) / 224
    width = abs(top_right[0] - bottom_left[0]) / 224
    height = abs(top_right[1] - bottom_left[1]) / 224

    with open(output_path +"/"+ clas+ "_"+ file[:-4] + ".txt", "w") as f:
        f.write(clas + " " + str(center_x) + " " + str(center_y) + " " + str(width) + " " + str(height))

    if width != 0.0 and height != 0.0:
      i+=1
    print(i , len(os.listdir(origin_path)))
    top_right = None
    bottom_left = None
    temp_bottom_left = None