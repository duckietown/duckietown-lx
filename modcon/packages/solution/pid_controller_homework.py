from typing import Tuple

import numpy as np
import yaml
import os

def PIDController(
    v_0: float, y_ref: float, y_hat: float, prev_e_y: float, prev_int_y: float, delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing lateral control.

    Args:
        v_0:        linear Duckiebot speed (constant).
        y_ref:      target y coordinate.
        y_hat:      the current estimated y.
        prev_e_y:   tracking error at previous iteration.
        prev_int_y: previous integral error term.
        delta_t:    time interval since last call.

    Returns:
        v_0:        linear velocity of the Duckiebot
        omega:      angular velocity of the Duckiebot
        e:          current tracking error (automatically becomes prev_e_y at next iteration).
        e_int:      current integral error (automatically becomes prev_int_y at next iteration).
    """

    # Read PID gains from file
    script_dir = os.path.dirname(__file__)
    file_path = script_dir + "/GAINS.yaml"

    with open(file_path) as f:
        gains = yaml.full_load(f)
        f.close()
    
    kp = gains['kp']
    kd = gains['kd']
    ki = gains['ki']

    # ------------- DEFINE YOUR PID FUNCTION BELOW ---------
    # PID errors computation
    e = y_ref-y_hat

    e_y_der = (e-prev_e_y)/delta_t

    e_y_int = e*delta_t + prev_int_y
    # e_int = max(min(e_y_int,1),-1)
    e_int = e_y_int

    # control action computation
    omega = kp*e + kd*e_y_der + ki*e_int

    # Hint: print for debugging
    #print(f"\n\nDelta time : {delta_t} \nE : {e} \nE int : {e_int} \nPrev e : {prev_e_y} \nPrev e_int : {prev_int_y} \nY hat : {y_hat} \nY ref : {y_ref} \nomega : {omega} \n")

    # ---
    
    return v_0, omega, e, e_int

