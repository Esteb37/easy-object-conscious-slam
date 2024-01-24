import os
import cv2
import numpy as np
import time

classes = ["ball","dog","gnome","table","chair","door","duck","person","extinguisher","flowers","shelf"]  # "2"

colors = np.random.uniform(0, 255, size=(len(classes), 3))

# cambiar para tu path
images_path = "/home/esteb37/ocslam/run/ocslam/resource/outputs/"

# igual cambiar
output_path = "/home/esteb37/ocslam/run/ocslam/resource/outputs"

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

for file in os.listdir(images_path):
  name = images_path + "/" + file

  if os.path.isfile(output_path +"/"+ file[:-4] + ".txt"):
    continue

  img = cv2.imread(name)
  if img is not None:

    coordinates = []
    current_class = 0
    print(classes[current_class])
    while current_class < len(classes):
      while top_right is None or bottom_left is None:
        copy = img.copy()
        if (top_right is not None and temp_bottom_left is not None):
            cv2.rectangle(copy, top_right, temp_bottom_left, colors[current_class], 2)
            cv2.putText(copy, classes[current_class], (top_right[0], top_right[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colors[current_class], 2)

        cv2.imshow("image", copy)
        if cv2.waitKey(1) == 13:
          current_class += 1
          if current_class == len(classes):
            break
          print(classes[current_class])

      if current_class == len(classes):
        break

      center_x = int((top_right[0] + bottom_left[0]) / 2) / 224
      center_y = int((top_right[1] + bottom_left[1]) / 2) / 224
      width = abs(top_right[0] - bottom_left[0]) / 224
      height = abs(top_right[1] - bottom_left[1]) / 224
      top_right = None
      bottom_left = None
      temp_bottom_left = None
      coordinates.append(str(current_class) + " " + str(center_x) + " " + str(center_y) + " " + str(width) + " " + str(height))
      img = copy.copy()


    with open(output_path +"/"+ file[:-4] + ".txt", "w") as f:
        for coordinate in coordinates:
            f.write(coordinate + "\n")