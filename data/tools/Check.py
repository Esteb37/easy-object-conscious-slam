import os
import cv2
import numpy as np

path ="/home/esteb37/ocslam/run/ocslam/resource/outputs"

images = [f for f in os.listdir(path) if f.endswith(".png")]


classes = ["ball","dog","gnome","table","chair","door","duck","person","extinguisher","flowers","shelf"]

colors = np.random.uniform(0, 255, size=(len(classes), 3))


for image_file in images:
  coord_file = image_file[:-4] + ".txt"
  image = cv2.imread(os.path.join(path, image_file))
  print(image.shape)
  with open(os.path.join(path, coord_file), "r") as f:
    # draw all the bounding boxes
    for line in f.readlines():
      line = line.split(" ")
      class_number = int(line[0])
      center_x = int(float(line[1]) * image.shape[1])
      center_y = int(float(line[2]) * image.shape[0])
      width = int(float(line[3]) * image.shape[1])
      height = int(float(line[4]) * image.shape[0])
      top_right = (int(center_x + width / 2), int(center_y + height / 2))
      bottom_left = (int(center_x - width / 2), int(center_y - height / 2))
      cv2.rectangle(image, top_right, bottom_left, colors[class_number], 2)
      cv2.putText(image, classes[class_number], (top_right[0], top_right[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, colors[class_number], 2)


  cv2.imshow("image", image)
  cv2.waitKey(0)