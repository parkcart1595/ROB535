import casadi as ca
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
import time

def nmpc_controller():
    # Declare simulation constants
    T = # TODO: planning horizon in seconds, controls how far into the future we plan
    N = # TODO: number of control intervals, defines the resolution of our control actions
    h = T / N

    # system dimensions
    Dim_state = # TODO: Number of states: x-position, y-position, orientation, speed
    Dim_ctrl  = # TODO: Number of control inputs: acceleration and steering angle

    # additional parameters
    x_init = ca.MX.sym('x_init', (Dim_state, 1))  # initial condition, # the state should be position to the leader car
    v_leader = ca.MX.sym('v_leader',(2, 1))       # leader car's velocity w.r.t ego car
    v_des = ca.MX.sym('v_des')                    # desired speed for the ego vehicle
    delta_last = ca.MX.sym('delta_last')          # last steering angle (used for rate constraints)
    params = ca.vertcat(x_init, v_leader, v_des, delta_last)
    
    # Continuous dynamics model
    x_model = ca.MX.sym('xm', (Dim_state, 1))
    u_model = ca.MX.sym('um', (Dim_ctrl, 1))

    L_f = 1.0 # Car parameters, do not change
    L_r = 1.0 # Car parameters, do not change

    beta = # TODO: The angle at which the car moves sideways relative to its orientation

    xdot = # TODO: xdot describes how each state variable changes over time based on current state and control (x-position change, y-position change, orientation change, speed change)

    # Discrete time dynmamics model
    Func_dynmaics_dt = # TODO 
    
    # Declare model variables, note the dimension
    x = # TODO
    u = # TODO

    # Define the cost function (objective) components
    # These encourage the car to stay in its lane, follow the leader, and achieve desired speed
    P = # TODO
    L = # TODO

    Func_cost_terminal = ca.Function('P', [x_model, params], [P])
    Func_cost_running = ca.Function('Q', [x_model, u_model, params], [L])

    # state and control constraints
    state_ub = # TODO: Example: large bounds for position, tighter on lateral position
    state_lb = # TODO 
    ctrl_ub  = # TODO: Control limits for acceleration and steering angle
    ctrl_lb  = # TODO 
    
    # upper bound and lower bound
    ub_x = np.matlib.repmat(state_ub, N + 1, 1)
    lb_x = np.matlib.repmat(state_lb, N + 1, 1)

    ub_u = np.matlib.repmat(ctrl_ub, N, 1)
    lb_u = np.matlib.repmat(ctrl_lb, N, 1)

    ub_var = np.concatenate((ub_u.reshape((# TODO, 1)), ub_x.reshape((# TODO, 1))))
    lb_var = np.concatenate((lb_u.reshape((# TODO, 1)), lb_x.reshape((# TODO, 1))))

    # dynamics constraints: x[k+1] = x[k] + f(x[k], u[k]) * dt
    # This enforces the system's discrete dynamics, meaning each next state is based on the current state and control.
    cons_dynamics = []
    ub_dynamics = np.zeros((# TODO, 1))
    lb_dynamics = np.zeros((# TODO, 1))
    for k in range(N):
        # Fx represents the calculated state at the next time step based on the dynamics model.
        # For each state variable (e.g., x-position, y-position, orientation, speed), we add a constraint.
        # This loop means that the computed next state (Fx) matches the predicted state (x[:, k+1]).
        Fx = Func_dynmaics_dt(x[:, k], u[:, k], params)  
        # TODO


    # state constraints: G(x) <= 0
    cons_state = []
    for k in range(N):
        #### collision avoidance:
        cons_state.append(# TODO)

        #### Maximum lateral acceleration ####
        dx = (x[:, k+1] - x[:, k]) / h  # Change in state over time step
        ay = # TODO: Compute the lateral acc (change in orientation * speed) using the hints
        
        gmu = (0.5 * 0.6 * 9.81)
        # Upper and lower bound on lateral acceleration
        cons_state.append(# TODO: Define upper bound on lateral acceleration)
        cons_state.append(# TODO: Define lower bound on lateral acceleration)

        #### lane keeping ####
        # Upper and lower bound on lateral position
        cons_state.append(# TODO)
        cons_state.append(# TODO)

        #### steering rate ####
        if k >= 1:
            d_delta = # TODO: Difference between current and previous steering angle 

            # Constraint steering rate to ensure smooth changes, scaled by time step `h` for discretization.
            # Upper and lower bound on steering rate
            cons_state.append(# TODO)
            cons_state.append(# TODO)
        else:
            d_delta = # TODO: for the first input, given d_last from param
            cons_state.append(# TODO)
            cons_state.append(# TODO)

    ub_state_cons = np.zeros((len(cons_state), 1))
    lb_state_cons = np.zeros((len(cons_state), 1)) - 1e5

    # cost function: # NOTE: You can also hard code everything here
    J = Func_cost_terminal(x[:, -1], params)
    for k in range(N):
        J = J + Func_cost_running(x[:, k], u[:, k], params)

    # initial condition as parameters
    cons_init = [x[:, 0] - x_init]
    ub_init_cons = np.zeros((Dim_state, 1))
    lb_init_cons = np.zeros((Dim_state, 1))
    
    # Define variables for NLP solver
    vars_NLP   = ca.vertcat(u.reshape((Dim_ctrl * N, 1)), x.reshape((Dim_state * (N+1), 1)))
    cons_NLP = cons_dynamics + cons_state + cons_init
    cons_NLP = ca.vertcat(*cons_NLP)
    lb_cons = np.concatenate((lb_dynamics, lb_state_cons, lb_init_cons))
    ub_cons = np.concatenate((ub_dynamics, ub_state_cons, ub_init_cons))

    # Create an NLP solver
    prob = {"x": vars_NLP, "p":params, "f": J, "g":cons_NLP}
    
    return prob, N, vars_NLP.shape[0], cons_NLP.shape[0], params.shape[0], lb_var, ub_var, lb_cons, ub_cons
