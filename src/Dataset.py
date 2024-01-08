import cv2
import os
import numpy as np

image_directory = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/images"
label_directory = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/outputs"
dataset_directory = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/dataset"

# loop through all folders of image_directory
folders = [folder for folder in os.listdir(image_directory) if not "." in folder]



for folder in folders:
  if folder == "Door" or folder == "Chair":
    continue

  # add all paths of images to a list
  files = [file for file in os.listdir(os.path.join(image_directory, folder)) if file.endswith(('.jpg', '.jpeg', '.png'))]
  #shuffle
  np.random.shuffle(files)
  total = len(files)
  train = int(total*0.8)
  test = total - train

  for i in range(train):
    try:
      image_path = os.path.join(image_directory, folder, files[i])
      label_path = os.path.join(label_directory, folder+"_"+files[i].split(".")[0] + ".txt")
      dataset_path = os.path.join(dataset_directory, "train", folder+"_"+files[i])
      dataset_label_path = os.path.join(dataset_directory, "train", folder+"_"+files[i].split(".")[0] + ".txt")

      os.rename(image_path, dataset_path)
      os.rename(label_path, dataset_label_path)


    except Exception as e:

      pass

  for i in range(train, total):
    try:
      image_path = os.path.join(image_directory, folder, files[i])
      label_path = os.path.join(label_directory, files[i].split(".")[0] + ".txt")
      dataset_path = os.path.join(dataset_directory, "test", folder+"_"+files[i])
      dataset_label_path = os.path.join(dataset_directory, "test", folder+"_"+files[i].split(".")[0] + ".txt")
      os.rename(image_path, dataset_path)
      os.rename(label_path, dataset_label_path)
    except:
      pass
