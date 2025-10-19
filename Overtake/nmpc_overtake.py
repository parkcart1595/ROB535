import casadi as ca
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
import time

def nmpc_controller():
    # Declare simulation constants
    T = 4.0
    N = 40
    h = T / N

    # system dimensions
    Dim_state = 4
    Dim_ctrl  = 2

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

    beta = ca.arctan((L_r / (L_r + L_f)) * ca.arctan(u_model[1]))

    v_x_leader = v_leader[0]
    
    xdot = ca.vertcat(
        x_model[3] * ca.cos(x_model[2] + beta) - v_x_leader, # dx/dt
        x_model[3] * ca.sin(x_model[2] + beta),             # dy/dt
        (x_model[3] / L_r) * ca.sin(beta),                  # dpsi/dt
        u_model[0]                                          # dv/dt
    )
    
    # Discrete time dynmamics model
    Func_dynmaics_dt = ca.Function('f_ct', [x_model, u_model, params], [xdot])
    
    # Declare model variables, note the dimension
    x = ca.MX.sym('x', (Dim_state, N + 1))
    u = ca.MX.sym('u', (Dim_ctrl, N))

    w_y_T = 20.0     # Terminal lateral position
    w_psi_T = 10.0   # Terminal yaw angle
    w_v_T = 5.0      # Terminal speed tracking
    
    w_v = 1.0        # Running speed tracking
    w_a = 0.1        # Running acceleration input
    w_delta = 0.5    # Running steering input
    
    v_des_param = params[6]
    
    # Define the cost function (objective) components
    # These encourage the car to stay in its lane, follow the leader, and achieve desired speed
    P = w_y_T * x_model[1]**2 + \
        w_psi_T * x_model[2]**2 + \
        w_v_T * (x_model[3] - v_des_param)**2

    # L = # TODO (Running Cost - FAQ #2 참조, C1, C4)
    L = w_v * (x_model[3] - v_des_param)**2 + \
        w_a * u_model[0]**2 + \
        w_delta * u_model[1]**2

    Func_cost_terminal = ca.Function('P', [x_model, params], [P])
    Func_cost_running = ca.Function('Q', [x_model, u_model, params], [L])

    # state and control constraints
    state_ub = np.array([ca.inf, ca.inf, ca.inf, ca.inf])
    state_lb = np.array([-ca.inf, -ca.inf, -ca.inf, -ca.inf])
    ctrl_ub  = np.array([4.0, 0.6])
    ctrl_lb  = np.array([-10.0, -0.6])
    
    # upper bound and lower bound
    ub_x = np.matlib.repmat(state_ub, N + 1, 1)
    lb_x = np.matlib.repmat(state_lb, N + 1, 1)

    ub_u = np.matlib.repmat(ctrl_ub, N, 1)
    lb_u = np.matlib.repmat(ctrl_lb, N, 1)

    ub_var = np.concatenate((ub_u.reshape((Dim_ctrl * N, 1)), ub_x.reshape((Dim_state * (N+1), 1))))
    lb_var = np.concatenate((lb_u.reshape((Dim_ctrl * N, 1)), lb_x.reshape((Dim_state * (N+1), 1))))

    # dynamics constraints: x[k+1] = x[k] + f(x[k], u[k]) * dt
    # This enforces the system's discrete dynamics, meaning each next state is based on the current state and control.
    cons_dynamics = []
    ub_dynamics = np.zeros((Dim_state * N, 1))
    lb_dynamics = np.zeros((Dim_state * N, 1))
    for k in range(N):
        # Fx represents the calculated state at the next time step based on the dynamics model.
        # For each state variable (e.g., x-position, y-position, orientation, speed), we add a constraint.
        # This loop means that the computed next state (Fx) matches the predicted state (x[:, k+1]).
        Fx = Func_dynmaics_dt(x[:, k], u[:, k], params)  
        for j in range(Dim_state):
            cons_dynamics.append(x[j, k+1] - Fx[j])


    # state constraints: G(x) <= 0
    cons_state = []
    for k in range(N):
        #### collision avoidance:
        cons_state.append(1.0 - (x[0, k] / 30.0)**2 - (x[1, k] / 2.0)**2)

        #### Maximum lateral acceleration ####
        dx = (x[:, k+1] - x[:, k]) / h  # Change in state over time step
        
        v_k = x[3, k]
        delta_k = u[1, k]
        beta_k = ca.arctan((L_r / (L_r + L_f)) * ca.arctan(delta_k))
        ay = (v_k**2 / L_r) * ca.sin(beta_k)
        
        gmu = (0.5 * 0.6 * 9.81)
        # Upper and lower bound on lateral acceleration
        cons_state.append(ay - gmu)
        cons_state.append(-ay - gmu)

        #### lane keeping ####
        # Upper and lower bound on lateral position
        cons_state.append(x[1, k] - 3.0)
        cons_state.append(-1.0 - x[1, k])

        rate_max_h = 0.6 * h
        #### steering rate ####
        if k >= 1:
            d_delta = u[1, k] - u[1, k-1]

            # Constraint steering rate to ensure smooth changes, scaled by time step `h` for discretization.
            # Upper and lower bound on steering rate
            cons_state.append(d_delta - rate_max_h)
            cons_state.append(-d_delta - rate_max_h)
        else:
            delta_last_param = params[7]
            d_delta = delta_last_param = params[7]
            cons_state.append(d_delta - rate_max_h)
            cons_state.append(-d_delta - rate_max_h)

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
