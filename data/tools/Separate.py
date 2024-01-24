import cv2
import os

object_name = "Ball"
image_directory = "/mnt/c/Users/esteb/OneDrive/Documents/Ulm/Homework/Learning/ros_ws/src/ocslam/resource/images/all"

parent_directory = os.path.dirname(image_directory)

print(image_directory)
print(parent_directory)

def save_image(image, count):
    image_filename = f"{count}.png"

    # if there is a folder with name "object_name" in the image directory, save the image there
    if os.path.exists(os.path.join(parent_directory, object_name)):
        image_path = os.path.join(parent_directory, object_name, image_filename)
    else:
        os.makedirs(os.path.join(parent_directory, object_name))
        image_path = os.path.join(parent_directory, object_name, image_filename)


    cv2.imwrite(image_path, image)
    print(f"Image saved: {image_path}")

def main():
    images = [file for file in os.listdir(image_directory) if file.endswith(('.jpg', '.jpeg', '.png'))]
    image_count = 0

    #sort
    images.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))

    for image_file in images:
        image_path = os.path.join(image_directory, image_file)
        image = cv2.imread(image_path)
        cv2.imshow("Image Viewer", image)
        key = cv2.waitKey(0)

        if key == ord('s'):  # Press 's' to save image
            save_image(image, image_count)
            image_count += 1
        elif key == 27:  # Press 'Esc' to exit
            print("User exited")
            break

if __name__ == "__main__":

  main()
