from typing import Tuple

import numpy as np


def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    res = np.zeros(shape=shape, dtype="float32")
    # these are random values
    res [:,:int(np.floor(shape[1]/4))]=1
    res [:,int(np.floor(shape[1]/4)):2*int(np.floor(shape[1]/4))]=-1
    # ---
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    # TODO: write your function instead of this one
    res = np.zeros(shape=shape, dtype="float32")
    # these are random values
    res [:,2*int(np.floor(shape[1]/4)):3*int(np.floor(shape[1]/4))]=1
    res [:,3*int(np.floor(shape[1]/4)):]=-1
    # ---
    return res
