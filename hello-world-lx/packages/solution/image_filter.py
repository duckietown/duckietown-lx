import cv2
import numpy as np

lower_hsv = np.array([10, 25, 20])
upper_hsv = np.array([30, 100, 255])


def highlight_centerline(image_rgb: np.ndarray) -> np.ndarray:
    """Returns a 2D array representing an image with a color filter applied"""
    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # TODO: uncomment the return line below
    # return mask