import json
import os
import cv2
import numpy as np
from tqdm import tqdm
from typing import Dict
from utils import xminyminxmaxymax2xywfnormalized, train_test_split, makedirs, runp


def load_annotations(dataset_dir) -> Dict:
    """
        Args:
            dataset_dir: Directory containing the image dataset.
        Return:
            annotations: Loaded image annotations containing objects and bounding boxes.
    """
    with open(f"{dataset_dir}/annotation/final_anns.json") as anns:
        annotations = json.load(anns)
        return annotations

def save_img(img, boxes, classes):
    global npz_index
    cv2.imwrite(f"{DATASET_DIR}/images/real_{npz_index}.jpg", img)
    with open(f"{DATASET_DIR}/labels/real_{npz_index}.txt", "w") as f:
        for i in range(len(boxes)):
            f.write(f"{classes[i]} "+" ".join(map(str,boxes[i]))+"\n")
    npz_index += 1
    all_image_names.append(f"real_{npz_index}")
