from typing import Tuple

import numpy as np


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    """
        Args:
            ticks: Current tick count from the encoders.
            prev_ticks: Previous tick count from the encoders.
            resolution: Number of ticks per full wheel rotation returned by the encoder.
        Return:
            rotation_wheel: Rotation of the wheel in radians.
            ticks: current number of ticks.
    """

    # Calculate the total rotation of the motor shaft, given the wheel encoder ticks
    delta_ticks = ticks-prev_ticks

    # Assuming no wheel slipping
    dphi = 2*np.pi*delta_ticks/resolution
    
    #Dticks = prev_ticks + int(np.random.uniform(0, 10))
    #dphi = np.random.random()

    # ---
    return dphi, ticks


def pose_estimation(
        R: float,
        baseline: float,
        x_prev: float,
        y_prev: float,
        theta_prev: float,
        delta_phi_left: float,
        delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x:                  estimated x coordinate
        y:                  estimated y coordinate
        theta:              estimated heading
    """

    # Use the odometry model to estimate the Duckiebot's pose

    w = [R, 2*R / baseline, 1]

    x = np.array(
        [
            [
                (delta_phi_left + delta_phi_right) * np.cos(theta_prev) / 2,
                (delta_phi_left + delta_phi_right) * np.sin(theta_prev) / 2,
                0,
            ],
            [0, 0, (delta_phi_right - delta_phi_left) / 2],
            [x_prev, y_prev, theta_prev],
        ]
    )

    x_curr, y_curr, theta_curr = np.array(w).dot(x)

    # x_curr = np.random.random()
    # y_curr = np.random.random()
    # theta_curr = np.random.random()
    # ---
    return x_curr, y_curr, theta_curr
