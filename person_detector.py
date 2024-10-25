#! /usr/bin/env python3

import supervision as sv
import cv2

from inference_sdk import InferenceHTTPClient

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key= #add API key here
)

img_path = #add path here
model = "people-detection-o4rdr/7"
image = cv2.imread(img_path)

result = CLIENT.infer(image, model_id=model)
print(result)

labels = [item["class"] for item in result["predictions"]]

detections = sv.Detections.from_roboflow(result)

label_annotator = sv.LabelAnnotator()
bounding_box_annotator = sv.BoxAnnotator()

image = cv2.imread(img_path)

annotated_image = bounding_box_annotator.annotate(
    scene=image, detections=detections)

cv2.imwrite("save_img.png", annotated_image)
