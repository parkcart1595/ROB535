from sim import *
from utils import *

def nmpc_controller(kappa_table = None):
    T = # TODO: planning horizon. keeping it very long helps in debugging but not necessary
    N = # TODO: number of control intervals, initializing this incorrectly may cause matrix multiplicative errors
    h = T / N
    ###################### Modeling Start ######################
    # system dimensions
    Dim_state = 6
    Dim_ctrl  = 2
    Dim_aux   = 4 # (F_yf, F_yr, F_muf (slack front), F_mur (slack rear))

    xm = ca.MX.sym('xm', (Dim_state, 1))
    um = ca.MX.sym('um', (Dim_ctrl, 1))
    zm = ca.MX.sym('zm', (Dim_aux, 1))

    ## rename the control inputs
    Fx = um[0]
    delta = um[1]

    ## Air drag and Rolling Resistance
    Fd = param["Frr"] + param["Cd"] * xm[0]**2
    Fd = Fd * ca.tanh(- xm[0] * 100)
    Fb = 0.0
    ## 
    
    # hint: find useful functions in utils.py
    af, ar   = # TODO: Refer to the car simulator for a function that computes slip angles, which are essential for lateral force calculations
    Fzf, Fzr = # TODO: Refer to the car simulator to get the normal load distribution
    Fxf, Fxr = # TODO: Refer to the car simulator to get the tire force distribution

    Fyf = # TODO: Use the modified tire force model tire_model_ctrl()
    Fyr = # TODO: Use the modified tire force mode ltire_model_ctrl()

    dUx  = (Fxf * ca.cos(delta) - zm[0] * ca.sin(delta) + Fxr + Fd) / param["m"] + xm[2] * xm[1]
    dUy  = # TODO: Refer to car simulator, replace Fyf and Fyr with auxiliary variable
    dr   = # TODO: Refer to car simulator, replace Fyf and Fyr with auxiliary variable
    
    dx   = ca.cos(xm[5]) * xm[0] - ca.sin(xm[5]) * xm[1]
    dy   = ca.sin(xm[5]) * xm[0] + ca.cos(xm[5]) * xm[1]
    dyaw = xm[2]
    xdot = ca.vertcat(dUx, dUy, dr, dx, dy, dyaw)

    xkp1 = xdot * h + xm
    Fun_dynmaics_dt = ca.Function('f_dt', [xm, um, zm], [xkp1])

    # Enforce constraints for auxiliary variable z[0] = Fyf to match actual tire forces for consistency.
    alg  = ca.vertcat(# TODO, # TODO)
    Fun_alg = ca.Function('alg', [xm, um, zm], [alg])
    
    ###################### MPC variables ######################
    x = ca.MX.sym('x', (Dim_state, N + 1))
    u = ca.MX.sym('u', (Dim_ctrl, N))
    z = ca.MX.sym('z', (Dim_aux, N))
    p = ca.MX.sym('p', (Dim_state, 1))

    ###################### MPC constraints start ######################
    ## MPC equality constraints ##
    cons_dynamics = []
    for k in range(N):
        xkp1 = Fun_dynmaics_dt(x[:, k], u[:, k], z[:, k])
        Fy2  = Fun_alg(x[:, k], u[:, k], z[:, k])
        for j in range(Dim_state):
            cons_dynamics.append(x[j, k+1] - xkp1[j])
        for j in range(2):
            cons_dynamics.append(Fy2[j])
    
    ## MPC inequality constraints ##
    # G(x) <= 0
    cons_ineq = []

    ## state / inputs limits:
    ## Refer to section 5 of the notebook 
    for k in range(N):
        cons_ineq.append(# TODO: Minimal longitudinal speed)
        cons_ineq.append(# TODO: Engine power limits)
        cons_ineq.append(# TODO: Collision avoidance)

    ## friction cone constraints
    for k in range(N):
        Fx, delta = u[0, k], u[1, k]
        af, ar =       # TODO Refer to the car simulator for a function that computes slip angles, which are essential for lateral force calculations
        Fzf, Fzr =     # TODO Refer to the car simulator to get the normal load distribution
        Fxf, Fxr =     # TODO Refer to the car simulator to get the tire force distribution


        Fyf = # TODO: Use tire_model_ctrl or auxiliary variable z[0, k] z[2, k]
        Fyr = # TODO: Use tire_model_ctrl or auxiliary variable z[1, k] z[3, k]

        cons_ineq.append(# TODO: Front tire limits)
        cons_ineq.append(# TODO: Rear  tire limits)        

    ###################### MPC cost start ######################
    ## cost function design, you can use a desired velocity v_des for stage cost
    ## Refer to section 6 in the notebook for more details.
    J = 0.0
    J +=  # TODO: Terminal cost
    
    ## road tracking 
    for k in range(N):
        J += # TODO: Stage cost
 
    ## Excessive slip angle / friction
    for k in range(N):
        Fx = u[0, k]; delta = u[1, k]
        af, ar = get_slip_angle(# TODO)
        Fzf, Fzr = normal_load(Fx, param)
        Fxf, Fxr = chi_fr(Fx)

        xi = 0.85
        F_offset = 2000   ## A slacked ReLU function using only sqrt()
        Fyf_max_sq = (param["mu_f"] * Fzf)**2 - (0.999 * Fxf)**2
        Fyf_max_sq = (ca.sqrt( Fyf_max_sq**2 + F_offset) + Fyf_max_sq) / 2
        Fyf_max = ca.sqrt(Fyf_max_sq)

        ## Modified front slide sliping angle
        alpha_mod_f = ca.arctan(3 * Fyf_max / param["C_alpha_f"] * xi)

        Fyr_max_sq = (param["mu_f"] * Fzf)**2 - (0.999 * Fxf)**2
        Fyr_max_sq = (ca.sqrt( Fyr_max_sq**2 + F_offset) + Fyr_max_sq) / 2
        Fyr_max = ca.sqrt(Fyr_max_sq)

        ## Modified rear slide sliping angle
        alpha_mod_r = ca.arctan(3 * Fyr_max / param["C_alpha_r"] * xi)

        ## Limit friction penalty
        J = J + # TODO: Avoid front tire saturation
        J = J + # TODO: Avoid  rear tire saturation
        J = J + # TODO: Penalize slack variable for friction cone limits

    # Initial condition as parameters
    cons_init = [x[:, 0] - p]
    ub_init_cons = np.zeros((Dim_state, 1))
    lb_init_cons = np.zeros((Dim_state, 1))

    state_ub = np.array([ 1e2,  1e2,  1e2,  1e8,  1e8,  1e8])
    state_lb = np.array([-1e2, -1e2, -1e2, -1e8, -1e8, -1e8])
    
    ## Set the control limits for upper and lower bounds
    ctrl_ub  = np.array([ # TODO,   # TODO]) # (traction force, param["delta_max"])
    ctrl_lb  = np.array([ # TODO,   # TODO]) # (-traction force, -param["delta_max"])
    
    aux_ub   = np.array([ 1e5,  1e5,  1e5,  1e5])
    aux_lb   = np.array([-1e5, -1e5, -1e5, -1e5])

    lb_dynamics = np.zeros((len(cons_dynamics), 1))
    ub_dynamics = np.zeros((len(cons_dynamics), 1))

    lb_ineq = np.zeros((len(cons_ineq), 1)) - 1e9
    ub_ineq = np.zeros((len(cons_ineq), 1))

    ub_x = np.matlib.repmat(state_ub, N + 1, 1)
    lb_x = np.matlib.repmat(state_lb, N + 1, 1)
    ub_u = np.matlib.repmat(ctrl_ub, N, 1)
    lb_u = np.matlib.repmat(ctrl_lb, N, 1)
    ub_z = np.matlib.repmat(aux_ub, N, 1)
    lb_z = np.matlib.repmat(aux_lb, N, 1)

    lb_var = np.concatenate((lb_u.reshape((Dim_ctrl * N, 1)), 
                             lb_x.reshape((Dim_state * (N+1), 1)),
                             lb_z.reshape((Dim_aux * N, 1))
                             ))

    ub_var = np.concatenate((ub_u.reshape((Dim_ctrl * N, 1)), 
                             ub_x.reshape((Dim_state * (N+1), 1)),
                             ub_z.reshape((Dim_aux * N, 1))
                             ))

    vars_NLP   = ca.vertcat(u.reshape((Dim_ctrl * N, 1)), x.reshape((Dim_state * (N+1), 1)), z.reshape((Dim_aux * N, 1)))
    cons_NLP = cons_dynamics + cons_ineq + cons_init
    cons_NLP = ca.vertcat(*cons_NLP)
    lb_cons = np.concatenate((lb_dynamics, lb_ineq, lb_init_cons))
    ub_cons = np.concatenate((ub_dynamics, ub_ineq, ub_init_cons))

    prob = {"x": vars_NLP, "p":p, "f": J, "g":cons_NLP}

    return prob, N, vars_NLP.shape[0], cons_NLP.shape[0], p.shape[0], lb_var, ub_var, lb_cons, ub_cons