import numpy as np
import cvxpy as cp

def calc_Jacobian(x, u, param):
    """
    Calculates the discrete-time Jacobian matrices A_k and B_k for the bicycle model.
    The linearization is performed around the given state x and control u.
    """

    L_f = param["L_f"]
    L_r = param["L_r"]
    dt   = param["h"]

    psi = x[2]
    v   = x[3]
    delta = u[1]
    a   = u[0]

    # Jacobian of the system dynamics
    A_c = np.zeros((4, 4))
    B_c = np.zeros((4, 2))

    # define slip angle beta
    beta = np.arctan(L_r / (L_f + L_r) * np.tan(delta))

    # derivative of beta with respect to delta
    d_beta_d_delta = (L_r / (L_f + L_r)) * (1 / (np.cos(delta)**2)) / (1 + (L_r / (L_f + L_r) * np.tan(delta))**2)

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    A_c[0, 2] = -v * np.sin(psi + beta)
    A_c[0, 3] = np.cos(psi + beta)
    A_c[1, 2] = v * np.cos(psi + beta)
    A_c[1, 3] = np.sin(psi + beta) 
    A_c[2, 3] = np.sin(beta) / L_r #

    # Derivatives with respect to control inputs
    # Column 1: acceleration 'a'
    B_c[3, 0] = 1.0
    
    # Column 2: steering 'delta'
    B_c[0, 1] = -v * np.sin(psi + beta) * d_beta_d_delta
    B_c[1, 1] = v * np.cos(psi + beta) * d_beta_d_delta
    B_c[2, 1] = v * np.cos(beta) / L_r * d_beta_d_delta #

    # Discrete-time Jacobians using explicit Euler
    # A_k = I + dt * Ac
    # B_k = dt * Bc
    A = np.eye(4) + dt * A_c
    B = dt * B_c

    # A = ...
    # B = ...

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

    return [A, B]

def LQR_Controller(x_bar, u_bar, x0, param):
    """
    Solves the time-varying LQR problem for trajectory tracking.
    """
    len_state = x_bar.shape[0] # horizon length N+1
    len_ctrl  = u_bar.shape[0] # horizon length N
    dim_state = x_bar.shape[1]
    dim_ctrl  = u_bar.shape[1]

    n_u = len_ctrl * dim_ctrl
    n_x = len_state * dim_state
    n_var = n_u + n_x

    n_eq  = dim_state * len_ctrl # dynamics
    n_ieq = dim_ctrl * len_ctrl  # input constraints

    
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    Q = np.diag([10.0, 10.0, 1.0, 1.0])

    R = np.diag([0.1, 0.1])

    Pt = Q

    # Define optimization variables for the entire horizon
    delta_s_k = cp.Variable((len_state, dim_state), name="delta_s_k")
    delta_u_k = cp.Variable((len_ctrl, dim_ctrl), name="delta_u_k")

    # --- 2. Construct Cost Function ---
    cost = 0
    for k in range(len_ctrl):
        # Sum of state and control costs over the horizon
        cost += cp.quad_form(delta_s_k[k, :], Q) + cp.quad_form(delta_u_k[k, :], R)
    
    # Add terminal cost
    cost += cp.quad_form(delta_s_k[len_ctrl, :], Pt)

    # --- 3. Construct Constraints ---
    
    constraints = []
    
    # Initial state constraint: deta_s_0 = s_actual - s_reference
    constraints.append(delta_s_k[0, :] == x0 - x_bar[0, :])
    
    # Dynamics constraints for k = 0 to N-1
    for k in range(len_ctrl):
        # linearized dynamics: delta_s_{k+1} = A_k * delta_s_k + B_k * delta_u_k
        A_k, B_k = calc_Jacobian(x_bar[k, :], u_bar[k, :], param)
        constraints.append(delta_s_k[k+1, :] == A_k @ delta_s_k[k, :] + B_k @ delta_u_k[k, :])

    # --- 4. Define and Solve the Problem ---
    
    prob = cp.Problem(cp.Minimize(cost), constraints)
    # Using OSQP solver as it's typically fast for this type of problem
    prob.solve(solver=cp.OSQP, verbose=False)
    
    if prob.status != cp.OPTIMAL:
        print("CVXPY problem could not be solved.")

    # The actual control to apply is the first optimal control deviation
    # added to the reference control input at the current step.
    u_act = delta_u_k.value[0, :] + u_bar[0, :]

    # Define and solve the CVXPY problem.
    # x = cp.Variable(n_var)
    # prob = cp.Problem(...)
    # prob.solve(verbose=False, max_iter = 10000)


    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################

    return u_act

def CMPC_Controller(x_bar, u_bar, x0, param):
    len_state = x_bar.shape[0]
    len_ctrl  = u_bar.shape[0]
    dim_state = x_bar.shape[1]
    dim_ctrl  = u_bar.shape[1]
    
    n_u = len_ctrl * dim_ctrl
    n_x = len_state * dim_state
    n_var = n_u + n_x

    n_eq  = dim_state * len_ctrl # dynamics
    n_ieq = dim_ctrl * len_ctrl # input constraints

    a_limit = param["a_lim"]
    delta_limit = param["delta_lim"]
    
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    
    # define the parameters
    # Q = np.eye(4)  * ...
    # R = np.eye(2)  * ...
    # Pt = np.eye(4) * ...
    
    # define the cost function
    # P = ...
    # q = ...
    
    # define the constraints
    # A = ...
    # b = ...
    # G = ...
    # ub = ...
    # lb = ...

    # Define and solve the CVXPY problem.
    # x = cp.Variable(n_var)
    # prob = cp.Problem(...)
    # prob.solve(verbose=False, max_iter = 10000)

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    
    # u_act = x.value[n_x:n_x + dim_ctrl] + u_bar[0, :]
    u_act = u_bar[0,:]
    return u_act