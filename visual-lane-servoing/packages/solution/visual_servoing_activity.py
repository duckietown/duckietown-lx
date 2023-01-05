from typing import Tuple

import numpy as np
import cv2


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """
    h, w, = shape
    steer_matrix_left = np.zeros((h,w))
    steer_matrix_left[:,:int(w/2)] = -0.005
    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right: The steering (angular rate) matrix for Braitenberg-like control
                            using the masked right lane markings (numpy.ndarray)
    """
    h, w, = shape
    steer_matrix_right = np.zeros((h,w))
    steer_matrix_right[:,int(w/2):] = +0.002
    # ---
    return steer_matrix_right


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
        right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    h, w, _ = image.shape

    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    """
    STEP 1: Image smoothin with Gaussian kernel
    """ 
    # This reduces the noise in the image
    sigma = 3
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)

    """
    STEP 2: Compute the gradients magnitudes and phase of the image using the sobel operator
    """
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)

    """
    STEP 3: non-maximal suppression of the gradients magnitudes
    """
    threshold = 60

    mask_mag = (Gmag > threshold)

    """
    STEP 4: color masking
    """
    white_lower_hsv = np.array([0, 0, 125])         # CHANGE ME
    white_upper_hsv = np.array([179, 65, 255])   # CHANGE ME
    yellow_lower_hsv = np.array([20, 75, 150])        # CHANGE ME
    yellow_upper_hsv = np.array([30, 255, 255])  # CHANGE ME

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    """
    STEP 5: edge-based masking
    """
    # Let's create masks for the left- and right-halves of the image
    width = w
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0

    # Generate a mask that identifies pixels based on the sign of their x-derivative
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)

    """
    STEP 6: generate final masks
    """
    mask_left_edge = mask_left * mask_mag * mask_yellow * mask_sobelx_neg * mask_sobely_neg
    mask_right_edge = mask_right * mask_mag * mask_white * mask_sobelx_pos * mask_sobely_neg

    left_masked_img = mask_left_edge*Gmag
    right_masked_img = mask_right_edge*Gmag

    return left_masked_img, right_masked_img