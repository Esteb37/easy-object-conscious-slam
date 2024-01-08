import os
import cv2
import numpy as np

outputs = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/outputs"

images ="/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/images"

for file in os.listdir(outputs):
  class_name = file.split("_")[0]
  image_number = file.split("_")[1].split(".")[0]
  image = cv2.imread(os.path.join(images, class_name,image_number + ".png"))
  # read the file, get coordinates, width and height and draw the rectangle

  with open(os.path.join(outputs, file), "r") as f:
    content = f.read()
    content = content.split(" ")
    center_x = float(content[1])
    center_y = float(content[2])
    width = float(content[3])
    height = float(content[4])
    top_right = (int((center_x + width/2)*224), int((center_y + height/2)*224))


    bottom_left = (int((center_x - width/2)*224), int((center_y - height/2)*224))
    cv2.rectangle(image, top_right, bottom_left, (0, 255, 0), 2)
    #write class name and image number above the rectangle
    cv2.putText(image, class_name+" "+str(image_number), (bottom_left[0], bottom_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)

    cv2.imshow("image", image)


    if cv2.waitKey(0) == 13:
      continue
