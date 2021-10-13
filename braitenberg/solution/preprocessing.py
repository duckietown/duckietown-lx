import numpy as np
import cv2

lower_hsv = np.array([0, 0, 0])
upper_hsv = np.array([179, 255, 255])


def preprocess(image_rgb: np.ndarray) -> np.ndarray:
    """ Returns a 2D array """
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    #     masked = cv2.bitwise_and(image, image, mask=mask)
    return mask
