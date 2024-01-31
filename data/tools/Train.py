from ultralytics import RTDETR
import os

path = "/home/esteb37/ocslam/resource/"

models = [#{"model_size": "n", "image_size": 640, "batch_size": 16},
          #{"model_size": "n", "image_size": 320, "batch_size": 32},
          {"model_size": "n", "image_size": 224, "batch_size": 16},
          #{"model_size": "s", "image_size": 640, "batch_size": 8},
          #{"model_size": "s", "image_size": 320, "batch_size": 16},
          #{"model_size": "n", "image_size": 224, "batch_size": 64},
          #{"model_size": "m", "image_size": 640, "batch_size": 8},
          #{"model_size": "m", "image_size": 320, "batch_size": 8},
          #{"model_size": "m", "image_size": 224, "batch_size": 16},
          #{"model_size": "l", "image_size": 640, "batch_size": 2},
          #{"model_size": "l", "image_size": 320, "batch_size": 4},
          #{"model_size": "l", "image_size": 224, "batch_size": 8},
          # {"model_size": "x", "image_size": 640, "batch_size": 2},
          #{"model_size": "x", "image_size": 320, "batch_size": 4},
          #{"model_size": "x", "image_size": 224, "batch_size": 16}
          ]

for model in models:
  model_size = model["model_size"]
  image_size = model["image_size"]
  batch_size = model["batch_size"]

  model =  RTDETR('rtdetr-l.pt')

  while batch_size >= 1:
    name = model_size + "_" + str(image_size) + "_" + str(batch_size)+"_rtdetr"
    try:
      results = model.train(data = path + str(image_size)+"_0.yaml", device=0, imgsz = image_size, verbose = True, plots = True, batch = batch_size, name = name, epochs = 500)
      break
    except Exception as e:
      batch_size = int(batch_size / 2)
      print("\n\n#############")
      print("Error in training " + name)
      print(e)
      print("#############\n\n")
