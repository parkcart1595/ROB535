from sim import *
from utils import *
import numpy as np

def nmpc_controller(kappa_table = None):
    T = 1.0 # planning horizon [s]
    N = 10  # control intervals
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
    # Fd = param["Frr"] + param["Cd"] * xm[0]**2
    # Fd = Fd * ca.tanh(- xm[0] * 100)
    Fd_base = param["Frr"] + param["Cd"] * xm[0]**2
    v_eps = 1.0
    sign_v = xm[0] / ca.sqrt(xm[0]**2 + v_eps**2)  # C¹ smooth sign(Ux)
    Fd = -Fd_base * sign_v
    Fb = 0.0
    ## 
    
    # hint: find useful functions in utils.py
    # slip angles / normal loads / longitudinal split
    af, ar   = get_slip_angle(xm[0], xm[1], xm[2], delta, param)
    Fzf, Fzr = normal_load(Fx, param)
    Fxf, Fxr = chi_fr(Fx)

    Fyf = tire_model_ctrl(af, Fzf, Fxf, param["C_alpha_f"], param["mu_f"])
    Fyr = tire_model_ctrl(ar, Fzr, Fxr, param["C_alpha_r"], param["mu_r"])

    dUx  = (Fxf * ca.cos(delta) - zm[0] * ca.sin(delta) + Fxr + Fd) / param["m"] + xm[2] * xm[1]
    dUy  = (zm[0] * ca.cos(delta) + Fxf * ca.sin(delta) + zm[1] + Fb) / param["m"] - xm[2] * xm[0]
    dr   = (param["L_f"] * (zm[0] * ca.cos(delta) + Fxf * ca.sin(delta)) - param["L_r"] * zm[1]) / param["Izz"]
    
    dx   = ca.cos(xm[5]) * xm[0] - ca.sin(xm[5]) * xm[1]
    dy   = ca.sin(xm[5]) * xm[0] + ca.cos(xm[5]) * xm[1]
    dyaw = xm[2]
    xdot = ca.vertcat(dUx, dUy, dr, dx, dy, dyaw)

    xkp1 = xdot * h + xm
    Fun_dynmaics_dt = ca.Function('f_dt', [xm, um, zm], [xkp1])

    # Enforce constraints for auxiliary variable z[0] = Fyf to match actual tire forces for consistency.
    alg  = ca.vertcat(zm[0] - Fyf,   # F_yf consistency
                      zm[1] - Fyr)   # F_yr consistency
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
    epsUx = 1e-2
    
    ## state / inputs limits:
    ## Refer to section 5 of the notebook 
    for k in range(N):
        cons_ineq.append(2.0 - x[0, k])  # <= 0
        # (b) Engine power: Fx <= Peng / max(Ux, eps)
        #cons_ineq.append(u[0, k] - (param["Peng"] / ca.fmax(x[0, k], epsUx)))  # <= 0
        denom = ca.sqrt(x[0, k]**2 + 1.0)   # ε=1.0 (필요시 0.5~2.0로 조정)
        cons_ineq.append(u[0, k] - (param["Peng"] / denom))  # <= 0
        # (c) Obstacle: 1 - ((x-500)/10)^2 - (y/10)^2 <= 0
        cons_ineq.append(1.0 - ((x[3, k] - 500.0) / 10.0)**2 - (x[4, k] / 10.0)**2)

    ## friction cone constraints
    for k in range(N):
        Fx, delta = u[0, k], u[1, k]
        af_k, ar_k    = get_slip_angle(x[0, k], x[1, k], x[2, k], delta, param)
        Fzf_k, Fzr_k  = normal_load(Fx, param)
        Fxf_k, Fxr_k  = chi_fr(Fx)

        Fyf_k, Fyr_k  = z[0, k], z[1, k]
        z_muf, z_mur  = z[2, k], z[3, k]

        # Front: Fyf^2 + Fxf^2 <= (mu_f Fzf)^2 + z_muf^2
        cons_ineq.append(Fyf_k**2 + Fxf_k**2 - (param["mu_f"] * Fzf_k)**2 - z_muf**2)
        # Rear : Fyr^2 + Fxr^2 <= (mu_r Fzr)^2 + z_mur^2
        cons_ineq.append(Fyr_k**2 + Fxr_k**2 - (param["mu_r"] * Fzr_k)**2 - z_mur**2)      

    ###################### MPC cost start ######################
    ## cost function design, you can use a desired velocity v_des for stage cost
    ## Refer to section 6 in the notebook for more details.
    
    # weights for case_0
    w_y, w_phi, w_r, w_Uy = 1.0, 2.0, 0.5, 1.0
    w_v, v_des            = 0.2, 200.0
    w_delta, w_du         = 0.1, 5.0
    w_mu, w_alpha         = 1e3, 5e3
    w_yT, w_phiT, w_xT    = 1.0, 4.0, 0.0
    
    # # weights for case_1
    # w_y, w_phi, w_r, w_Uy = 0.5, 2.0, 0.5, 1.0
    # w_v, v_des            = 0.05, 200.0
    # w_delta, w_du         = 0.1, 5.0
    # w_mu, w_alpha         = 1e4, 5e3
    # w_yT, w_phiT, w_xT    = 5.0, 4.0, 0.0
    
    J = 0.0
    J += w_yT * x[4, N]**2 + w_phiT * x[5, N]**2 - w_xT * x[3, N]  # Terminal cost
    
    ## road tracking 
    for k in range(N):
        J += (w_y   * x[4, k]**2
              + w_phi * x[5, k]**2
              + w_r   * x[2, k]**2
              + w_Uy  * x[1, k]**2
              + w_v   * (x[0, k] - v_des)**2
              + w_delta * u[1, k]**2)
        
        if k > 0:
            J += w_du * ((u[0, k] - u[0, k-1])**2 + (u[1, k] - u[1, k-1])**2)
 
    ## Excessive slip angle / friction
    for k in range(N):
        Fx = u[0, k]; delta = u[1, k]
        af, ar = get_slip_angle(x[0, k], x[1, k], x[2, k], delta, param)
        Fzf, Fzr = normal_load(Fx, param)
        Fxf, Fxr = chi_fr(Fx)

        xi = 0.85
        F_offset = 2000   ## A slacked ReLU function using only sqrt()
        Fyf_max_sq = (param["mu_f"] * Fzf)**2 - (0.999 * Fxf)**2
        Fyf_max_sq = (ca.sqrt( Fyf_max_sq**2 + F_offset) + Fyf_max_sq) / 2
        Fyf_max = ca.sqrt(Fyf_max_sq)

        ## Modified front slide sliping angle
        alpha_mod_f = ca.arctan(3 * Fyf_max / param["C_alpha_f"] * xi)

        Fyr_max_sq = (param["mu_r"] * Fzr)**2 - (0.999 * Fxr)**2
        Fyr_max_sq = (ca.sqrt(Fyr_max_sq**2 + F_offset) + Fyr_max_sq) / 2
        Fyr_max = ca.sqrt(Fyr_max_sq)

        ## Modified rear slide sliping angle
        alpha_mod_r = ca.arctan(3 * Fyr_max / param["C_alpha_r"] * xi)

        ## Limit friction penalty
        # J += w_alpha * ca.if_else(ca.fabs(af_k) >= alpha_mod_f,
        #                           (ca.fabs(af_k) - alpha_mod_f)**2, 0.0)
        # J += w_alpha * ca.if_else(ca.fabs(ar_k) >= alpha_mod_r,
        #                           (ca.fabs(ar_k) - alpha_mod_r)**2, 0.0)
        J += w_mu * (z[2, k]**2 + z[3, k]**2)

    # Initial condition as parameters
    # cons_init = [x[:, 0] - p]
    S0 = ca.diag(ca.DM([1/50.0, 1/10.0, 1/5.0, 1/100.0, 1/100.0, 1/10.0]))
    cons_init = [ca.mtimes(S0, (x[:, 0] - p))]
    ub_init_cons = np.zeros((Dim_state, 1))
    lb_init_cons = np.zeros((Dim_state, 1))

    state_ub = np.array([ 1e2,  1e2,  1e2,  1e8,  1e8,  1e8])
    state_lb = np.array([-1e2, -1e2, -1e2, -1e8, -1e8, -1e8])
    
    ## Set the control limits for upper and lower bounds
    ctrl_ub  = np.array([ param.get("Fx_max_soft", 3e4),  param["delta_max"]])
    ctrl_lb  = np.array([ 0.0, -param["delta_max"]])
    
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