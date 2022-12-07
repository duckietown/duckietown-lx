DATASET_DIR = "/code/object-detection/assets/duckietown_object_detection_dataset"
IMAGE_SIZE = 416
# this is the percentage of real data that will go into the training set (as opposed to the testing set)
REAL_TRAIN_TEST_SPLIT_PERCENTAGE = 0.8

import cv2
import numpy as np

mapping = {
    "house": "3deb34",
    "bus": "ebd334",
    "truck": "961fad",
    "duckie": "cfa923",
    "cone": "ffa600",
    "floor": "000000",
    "grass": "000000",
    "barrier": "000099"
}
mapping = {key: [int(h[i:i + 2], 16) for i in (0, 2, 4)] for key, h in mapping.items()}


def segmented_image_one_class(segmented_img, class_name):
    mask = np.all(segmented_img == mapping[class_name], axis=-1)
    return mask


def find_all_bboxes(mask):
    gray = mask.astype("uint8")
    gray[mask == True] = 255
    gray[mask == False] = 0
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    boxes = []
    for index, cnt in enumerate(contours):
        if hierarchy[0, index, 3] != -1:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        boxes.append([x, y, w + x, h + y])
    boxes = np.array(boxes)
    return boxes


def find_all_boxes_and_classes(segmented_img):
    classes = ["duckie", "cone", "truck", "bus"]
    all_boxes = []
    all_classes = []
    for i, class_name in enumerate(classes):
        mask = segmented_image_one_class(segmented_img, class_name)
        boxes = find_all_bboxes(mask)
        all_boxes.extend(list(boxes))
        classes = np.array([i] * boxes.shape[0])
        all_classes.extend(list(classes))
    return all_boxes, all_classes
