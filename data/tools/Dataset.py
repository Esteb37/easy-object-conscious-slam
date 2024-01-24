import cv2
import os
import numpy as np


path = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/outputs"

dataset_path = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/dataset"

images = [f for f in os.listdir(path) if f.endswith(".png")]

length = len(images)



widths = [640, 320, 224]
heights = [480, 240, 168]


np.random.shuffle(images)
train = images[:int(length * 0.8)]
test = images[int(length * 0.8):]

for i, width in enumerate(widths):
  height = heights[i]

  batch_path =  dataset_path + "/" + str(width) + "/" + str(2)
  train_path = batch_path + "/train"
  test_path = batch_path + "/test"

  #create a folder if nonexistent, if existent, delete it and create a new one
  if os.path.exists(batch_path):
    os.system("rm -rf " + batch_path)
    os.system("mkdir " + batch_path)
  else:
    os.system("mkdir " + batch_path)

  os.system("mkdir " + train_path)
  os.system("mkdir " + test_path)

  for image in train:
    img = cv2.imread(path + "/" + image)
    img = cv2.resize(img, (width, height))
    cv2.imwrite(train_path + "/" + image, img)
    os.system("cp " + path + "/" + image[:-4] + ".txt " + train_path+"/")

  for image in test:
    img = cv2.imread(path + "/" + image)
    img = cv2.resize(img, (width, height))
    cv2.imwrite(test_path + "/" + image, img)
    os.system("cp " + path + "/" + image[:-4] + ".txt " + test_path)

print("Done!")