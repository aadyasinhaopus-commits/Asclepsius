# import os
# from ultralytics import YOLO

# # 1. Load the YOLO-World model
# model = YOLO("yolov8s-worldv2.pt")

# # 2. Tell the model exactly what words to look for in the image
# model.set_classes(["human", "tree", "plant rows"])

# # 3. Define your image path (make sure the filename matches your file!)
# image_path = "/Users/eshajain/Downloads/img.jpeg"

# # 4. Run the prediction and tell it to save the annotated image
# results = model(image_path, save=True, project="/Users/eshajain/Desktop", name="YOLO_Detections")

# print("Detections completed successfully! Check your Desktop for the output folder.")

# import os
# from ultralytics import YOLO

# # 1. Load the worldv2 model
# model = YOLO("yolov8s-worldv2.pt")

# # 2. Use "person", and add the empty string background fix
# model.set_classes(["person", "tree", ""])

# # 3. Define your image path 
# image_path = "/Users/eshajain/Downloads/img.jpeg"

# # 4. Run prediction with a lower confidence threshold (conf=0.1)
# results = model(
#     image_path, 
#     save=True, 
#     conf=0.1, 
#     project="/Users/eshajain/Desktop", 
#     name="YOLO_Detections"
# )

# print("Check your Desktop for a folder named 'YOLO_Detections'!")

import cv2
from ultralytics import YOLO

# 1. Load the model
model = YOLO("yolov8s-worldv2.pt")

# 2. Set the objects you want to search for in real-time
model.set_classes(["person", "backpack", "cell phone"])

# 3. Run inference directly on webcam '0' using a live stream loop
# 'show=True' tells Ultralytics to open a video display window automatically
results = model.predict(source=0, stream=True, show=True, conf=0.15)
print("Webcam stream started! Click the video window and press 'q' to exit.")

# 4. Keep the stream active frame-by-frame
for r in results:
    # This loop forces python to keep reading the generator stream.
    # To close the window cleanly, press the 'q' key on your keyboard.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
