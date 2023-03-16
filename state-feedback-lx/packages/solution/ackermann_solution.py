import numpy as np
from typing import List

# Solution Exercise 1: Reachability

# AB = A @ B
# R = np.concatenate((B, AB), axis=1)
# r = np.linalg.matrix_rank(R)

# if r < 2:
#   print(f'The system is not reachable because the rank of R is: {r}.')
# else:
#   print(f'The system is reachable because rank of R is: {r}.')


# Solution Exercise 2: Ackermann's Formula

def acker_solution(A: np.array, B: np.array, poles: List[float]) -> np.array:
    AB = A @ B
    R = np.concatenate((B, AB), axis=1)
    R_inv = np.linalg.inv(R)
    gamma = np.array([[0, 1]]) @ R_inv
    
    p_1 = poles[0]*(-1)
    p_2 = poles[1]*(-1)
    ab = p_1 + p_2
    b = p_1*p_2
    p_cl = A @ A + ab*A + b*np.identity(2)
    
    K = gamma @ p_cl

    return K


# Solution Exercise 3: Feedforward Gain

def feedforward_kr_solution(A: np.array, B: np.array, C: np.array, K: np.array) -> np.array:
    ABK = A - B @ K
    ABK_inv = np.linalg.inv(ABK)
    den = C @ ABK_inv @ B
    kr = - 1/den

    return kr