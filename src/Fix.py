import os
import cv2
import numpy as np



#find all files named "DogDog..." and rename them to "Dog..."
outputs = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/outputs"

images ="/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/images"

#list only the folders in outputs, without .png files
classes = [f for f in os.listdir(images) if not f.endswith(".png")]

# go through files in outputs and get the class name, which is the first word before _
for file in os.listdir(outputs):
  filename = file.split("_")[0]
  class_number = classes.index(filename)
  # replace up to the first space in the content of the file with the class number
  with open(os.path.join(outputs, file), "r+") as f:
    content = f.read()
    content = content.replace(content[:content.find(" ")], str(class_number))
    f.seek(0)
    f.write(content)
    f.truncate()
