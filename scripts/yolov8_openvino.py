import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from copy import deepcopy

from openvino.runtime import Core
import numpy as np
import cv2
import yaml

# Model and classes file names
MODEL_NAME = "yolov8n.onnx"
CLASSES_YAML = "coco_classes.yaml"

# Read the content of the YAML file
with open(CLASSES_YAML) as f:
    yaml_content = f.read()

# Parse YAML data into a Python list
parsed_list = yaml.safe_load(yaml_content)

# Extract class names from the parsed YAML data
CLASSES = list(parsed_list["names"].values())

# Generate random colors for each class for bounding box visualization
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))


# Function to draw bounding boxes on the image
def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    label = f"{CLASSES[class_id]} ({confidence:.2f})"
    color = colors[class_id]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


# ObjectDetector class definition inheriting from rclpy Node
class ObjectDetector(Node):
    def __init__(self):
        super().__init__("yolov8_node")

        # Create a subscription to the "/image_raw" topic
        self.yolo_sub = self.create_subscription(
            Image, "/SpotArm/gripper_camera/image_color", self.image_cb, 1
        )

        # Initialize OpenVINO Core and load the YOLOv8 model
        core = Core()
        net = core.compile_model(f"{MODEL_NAME}", device_name="AUTO")
        self.output_node = net.outputs[0]  # yolov8n has only one output node
        self.ir = net.create_infer_request()

    # Callback function for processing incoming image messages
    def image_cb(self, msg):
        image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")

        frame = deepcopy(image)

        [height, width, _] = frame.shape
        length = max((height, width))
        image = np.zeros((length, length, 3), np.uint8)
        image[0:height, 0:width] = frame
        scale = length / 640

        blob = cv2.dnn.blobFromImage(
            image, scalefactor=1 / 255, size=(640, 640), swapRB=True
        )

        outputs = self.ir.infer(blob)[self.output_node]

        outputs = np.array([cv2.transpose(outputs[0])])
        rows = outputs.shape[1]
        outputs = outputs[0]

        boxes = []
        scores = []
        class_ids = []

        # Process detected objects from the model output
        for i in range(rows):
            classes_scores = outputs[i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(
                classes_scores
            )
            if maxScore >= 0.25:
                box_cx = outputs[i][0]
                box_cy = outputs[i][1]
                box_width = outputs[i][2]
                box_height = outputs[i][3]
                box = [
                    box_cx - (0.5 * box_width),
                    box_cy - (0.5 * box_height),
                    box_cx + (0.5 * box_width),
                    box_cy + (0.5 * box_height),
                ]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)

        # Perform non-maximum suppression on the detected boxes
        result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)

        detections = []
        # Process the boxes after non-maximum suppression and draw bounding boxes
        for i in range(len(result_boxes)):
            index = result_boxes[i]
            box = boxes[index]
            detection = {
                "class_id": class_ids[index],
                "class_name": CLASSES[class_ids[index]],
                "confidence": scores[index],
                "box": box,
                "scale": scale,
            }
            detections.append(detection)
            draw_bounding_box(
                frame,
                class_ids[index],
                scores[index],
                round(box[0] * scale),
                round(box[1] * scale),
                round(box[2] * scale),
                round(box[3] * scale),
            )

        # Display the annotated image
        cv2.imshow("YOLOv8 OpenVINO", frame)
        cv2.waitKey(1)


# Main function to initialize ROS and run the ObjectDetector node
def main():
    rclpy.init()
    rclpy.spin(ObjectDetector())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
