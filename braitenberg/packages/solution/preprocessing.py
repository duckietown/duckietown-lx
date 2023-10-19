import cv2
import numpy as np

lower_hsv = np.array([0.0001, 80, 40])
upper_hsv = np.array([25.7, 255, 255])


def preprocess(image_rgb: np.ndarray) -> np.ndarray:
    """Returns a 2D array"""
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    return mask
