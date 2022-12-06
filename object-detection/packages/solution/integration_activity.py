def DT_TOKEN():
    # TODO: change this to your duckietown token
    dt_token = "dt1-3nT8KSoxVh4MdLnE1Bq2mTkhRpbR35G8mmbjExKF7zGm6g4-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfYtfGWrfSxeihNZvYVNfNmnCBP28LeqDxL"
    return dt_token


def MODEL_NAME():
    # TODO: change this to your model's name that you used to upload it on google colab.
    # if you didn't change it, it should be "yolov5"
    return "yolov5"


def NUMBER_FRAMES_SKIPPED():
    # TODO: change this number to drop more frames
    # (must be a positive integer)
    return 0

    # NOTE: Sample solution
    return 1


def filter_by_classes(pred_class: int):
    """
    Args:
        pred_class: the class of a prediction
    """
    # Right now, this returns True for every object's class
    # TODO: Change this to only return True for duckies!
    # In other words, returning False means that this prediction is ignored.
    return True

    # NOTE: Solution
    """
    | Object    | ID    |
    | ---       | ---   |
    | Duckie    | 0     |
    | Cone      | 1     |
    | Truck     | 2     |
    | Bus       | 3     |
    """
    return pred_class == 0


def filter_by_scores(score):
    """
    Args:
        score: the confidence score of a prediction
    """
    # Right now, this returns True for every object's confidence
    # TODO: Change this to filter the scores, or not at all
    # (returning True for all of them might be the right thing to do!)
    return True


def filter_by_bboxes(bbox):
    """
    Args:
        bbox: is the bounding box of a prediction, in xyxy format
                This means the shape of bbox is (leftmost x pixel, topmost y, rightmost x, bottommost y)
    """
    # TODO: Like in the other cases, return False if the bbox should not be considered.
    return True
