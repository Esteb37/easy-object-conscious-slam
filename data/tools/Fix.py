import os
import cv2
import numpy as np



#find all files named "DogDog..." and rename them to "Dog..."
outputs_path = "/home/esteb37/ocslam/resource/outputs"


classes = ["ball","dog","gnome","table","chair","door","duck","person","extinguisher","flowers","shelf"]

#outputs are .txt
outputs = [f for f in os.listdir(outputs_path) if f.endswith(".txt")]

# go through files in outputs and get the class name, which is the first word before _
for file in outputs:
  new_text = ""
  with open(os.path.join(outputs_path, file), "r") as f:
    for line in f.readlines():
      line = line.split(" ")
      class_number = int(line[0])
      center_x = float(line[1]) * 224 / 640
      center_y = float(line[2]) * 224 / 480
      width = float(line[3]) * 224 / 640
      height = float(line[4]) * 224 / 480
      new_line = str(class_number) + " " + str(center_x) + " " + str(center_y) + " " + str(width) + " " + str(height) + "\n"
      new_text += new_line
  with open(os.path.join(outputs_path, file), "w") as f:
    f.write(new_text)
