from typing import Tuple

import numpy as np


def PIDController(
        v_0: float,
        theta_ref: float,
        theta_hat: float,
        prev_e: float,
        prev_int: float,
        delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing heading control.
    Args:
        v_0:        linear Duckiebot speed (given).
        theta_ref:  reference heading pose.
        theta_hat:  the current estimated theta.
        prev_e:     tracking error at previous iteration.
        prev_int:   previous integral error term.
        delta_t:    time interval since last call.
    Returns:
        v_0:     linear velocity of the Duckiebot
        omega:   angular velocity of the Duckiebot
        e:       current tracking error (automatically becomes prev_e_y at next iteration).
        e_int:   current integral error (automatically becomes prev_int_y at next iteration).
    """

    # PID GAINS DEFINITION

    k_p = 5     # Proportional gain: increasing this value will make
                # the controller quicker but will tend to overshoot

    k_d = 0.1     # Derivative gain: increasing this will damp the overshoot
                # but will make the response lower

    k_i = 0.2     # Integral gain: this term will remove steady-state error
    
    # Compute the error terms
    e = theta_ref-theta_hat
    e_der = (e-prev_e)/delta_t
    e_int = e*delta_t + prev_int

    omega = k_p*e + k_d*e_der + k_i*e_int
    # Hint: print for debugging
    # print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
    # ---
    return v_0, omega, e, e_int
