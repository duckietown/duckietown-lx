import cv2
import duckietown_code_utils as dtu
import numpy as np
from easy_algo import get_easy_algo_db
from easy_node.utils.timing import FakeContext
from ground_projection.ground_projection_interface import find_ground_coordinates
from ground_projection.segment import rectify_segments
from image_processing.more_utils import get_robot_camera_geometry
from line_detector2.image_prep import ImagePrep
from line_detector_interface import FAMILY_LINE_DETECTOR
from line_detector_interface.visual_state_fancy_display import vs_fancy_display
import matplotlib.pyplot as plt

line_detector_name = "baseline"
image_prep_name = "baseline"
anti_instagram_name = "baseline"
robot_name = "lab"
rcg = get_robot_camera_geometry(robot_name)


def detect_line_segments(img):
    dtu.check_isinstance(img, np.ndarray)

    algo_db = get_easy_algo_db()
    line_detector = algo_db.create_instance(FAMILY_LINE_DETECTOR, line_detector_name)
    image_prep = algo_db.create_instance(ImagePrep.FAMILY, image_prep_name)
    segment_list = image_prep.process(FakeContext(), img, line_detector, transform=None)
    jpg1 = vs_fancy_display(image_prep.image_cv, segment_list)

    jpg1 = jpg1.copy()
    jpg1[np.all(jpg1 == (255,255,255), axis=-1)] = (0,0,0)
    plt.imshow(cv2.cvtColor(np.float32(jpg1), cv2.COLOR_BGR2RGB))

    segment_list_rect = rectify_segments(rcg.rectifier, rcg.gpg, segment_list)

    sg = find_ground_coordinates(rcg.gpg, segment_list_rect)

    return sg
