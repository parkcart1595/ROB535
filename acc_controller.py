import numpy as np

def ACC_Controller(t, x, param):
    vd = param["vd"]
    v0 = param["v0"]
    m = param["m"]
    Cag = param["Cag"]
    Cdg = param["Cdg"]

    # cost function and constraints for the QP
    P = np.zeros((2,2))
    q = np.zeros([2, 1])
    A = np.zeros([5, 2])
    b = np.zeros([5])
    
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    # set the parameters
    lam = 10.0       # lambda for CLF (tracking performance)
    alpha = 1.0     # alpha for CBF (class K function)
    w = 100.0       # weight for the slack variable delta

    # Current state
    D = x[0]  # Distance to lead vehicle
    v = x[1]  # Ego vehicle velocity

    # Define the CLF (h) and CBF (B) functions
    # h: Control Lyapunov Function for tracking error
    h = 0.5 * (v - vd)**2

    # Clip the relative velocity at 0.
    v_rel = max(0, v - v0)

    # B: Control Barrier Function for safety
    # Note: The term (v0-v) can be negative, but it is squared, so it's fine.
    B = D - 1.8 * v - (0.5 * v_rel**2) / Cdg
    
    # construct the cost function for min(F_w^2 + w*delta^2)
    # The solver uses the form 0.5 * x'Px, so P must be scaled by 2.
    P[0, 0] = 2.0
    P[1, 1] = 2.0 * w
    
    # There are no linear terms in the cost function
    q[0, 0] = 0.0
    q[1, 0] = 0.0
    
    # construct the constraints (Ax <= b)
    
    # 1. Tracking Constraint (CLF): (v - vd)/m * F_w - delta <= -lambda * h
    A[0, 0] = (v - vd) / m
    A[0, 1] = -1.0
    b[0] = -lam * h

    # 2. Safety Constraint (CBF): (1/m)*(1.8 + (v-v0)/Cdg)*F_w <= (v0-v) + alpha*B
    A[1, 0] = (1.0 / m) * (1.8 + v_rel / Cdg)
    A[1, 1] = 0.0
    b[1] = (v0 - v) + alpha * B
    
    # 3. Input Constraint (Max Acceleration): F_w <= m * Cag
    A[2, 0] = 1.0
    A[2, 1] = 0.0
    b[2] = m * Cag

    # 4. Input Constraint (Max Deceleration): -F_w <= m * Cdg
    A[3, 0] = -1.0
    A[3, 1] = 0.0
    b[3] = m * Cdg

    # 5. Slack Variable Constraint: -delta <= 0
    A[4, 0] = 0.0
    A[4, 1] = -1.0
    b[4] = 0.0

    # construct the cost function
    # P = ...
    # q = ...
    
    # construct the constraints
    # A = ...
    # b = ...

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    
    return A, b, P, q