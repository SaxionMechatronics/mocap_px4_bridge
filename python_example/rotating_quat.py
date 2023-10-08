import numpy as np
import random
 
def quaternion_multiply(Q0,Q1):
    """
    Multiplies two quaternions.
 
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
 
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
 
    """
    # Extract the values from Q0
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
     
    # Extract the values from Q1
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
     
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
     
    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
     
    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
    return final_quaternion
     
q_FLU_FRD = np.array([0, 1, 0, 0]) # First quaternion
q = np.array([ 0.9673032, 0.0451397, 0.1236677, 0.2167793]) # Second quaternion
Q = quaternion_multiply(quaternion_multiply(q_FLU_FRD, q),q_FLU_FRD)
print("{0} x {1} x {2} = {3}".format(q_FLU_FRD, q, q_FLU_FRD, Q))
print("{0} Rotated from q_FLU^FLU to q_FRD^FRD:".format(q))
print("{0}".format(Q))
print("OR:")
print("{0}".format(-Q)) 
print("Remember: A quaternion q and -q represent the same rotation!")