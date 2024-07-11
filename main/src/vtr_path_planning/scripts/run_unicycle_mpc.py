import sys
from time import time
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from simulation_code import simulate_path_tracking

from unicycle_solver import solver, motion_model, alpha, N, step_horizon, n_states, n_controls


# specs
x_init = 0
y_init = 0.2
theta_init = 0

v_max = 1.5
v_min = -1.5
w_max = 0.5
w_min = -0.5
v_ref = 0.5*v_max

lin_acc_max = 1.00
ang_acc_max = 0.5


sim_time = 100      # simulation time


state_init = ca.DM([x_init, y_init, theta_init])        # initial state

def shift_timestep(step, t0, state_init, u, f, last_u):
    f_value = f(state_init, u[:, 0], last_u)
    next_state = ca.DM.full(state_init + (step * f_value))
    t0 = t0 + step
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )

    return t0, next_state, u0


def DM2Arr(dm):
    return np.array(dm.full())

path_x = np.linspace(0, 30, 1000)
path_y = 0.5*path_x + np.sin(2*np.pi*path_x/10)
path_mat = np.zeros((np.size(path_x), 3))
path_mat[:, 0] = path_x
path_mat[:, 1] = path_y
path_mat[:, 2] = np.arctan2(np.gradient(path_y), np.gradient(path_x))
path_p = np.zeros_like(path_x)
for i in range(1, np.size(path_p)):
    path_p[i] = path_p[i-1] + np.hypot(path_x[i] - path_x[i-1], path_y[i] - path_y[i-1]) + 0.25*np.abs(path_mat[i, 2] - path_mat[i-1, 2])


lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))

lbx[0: n_states*(N+1): n_states] = -ca.inf     # X lower bound
lbx[1: n_states*(N+1): n_states] = -ca.inf     # Y lower bound
lbx[2: n_states*(N+1): n_states] = -ca.inf     # theta lower bound

ubx[0: n_states*(N+1): n_states] = ca.inf      # X upper bound
ubx[1: n_states*(N+1): n_states] = ca.inf      # Y upper bound
ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound

lbx[n_states*(N+1)::2] = v_min                  # v lower bound for all V
ubx[n_states*(N+1)::2] = v_max                  # v upper bound for all V
lbx[n_states*(N+1)+1::2] = w_min                # v upper bound for all V
ubx[n_states*(N+1)+1::2] = w_max                # v upper bound for all V

#Motion Model Constraints
lbg = ca.DM.zeros((n_states*(N+1) + N + n_controls*N, 1))  # constraints lower bound
ubg = ca.DM.zeros((n_states*(N+1) + N + n_controls*N, 1))  # constraints upper bound

#Corridor Width constraints
lbg[n_states*(N+1):n_states*(N+1)+N] = -1.0
ubg[n_states*(N+1):n_states*(N+1)+N] = 1.0

#Acceleration constraints
lbg[n_states*(N+1)+N::2] = -lin_acc_max * step_horizon
ubg[n_states*(N+1)+N::2] = lin_acc_max * step_horizon
lbg[n_states*(N+1)+N+1::2] = -ang_acc_max * step_horizon
ubg[n_states*(N+1)+N+1::2] = ang_acc_max * step_horizon

print(lbx.shape)

args = {
    'lbg': lbg,
    'ubg': ubg,  
    'lbx': lbx,
    'ubx': ubx
}

t0 = 0

t = ca.DM(t0)

u0 = ca.DM.zeros((n_controls, N))  # initial control
X0 = ca.repmat(state_init, 1, N+1)         # initial state full


mpc_iter = 0
cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])
last_u = u0[:, 0]
state_target = path_mat[-1, :]

###############################################################################

if __name__ == '__main__':
    main_loop = time()  # return time in sec
    p_state = 0
    while (ca.norm_2(state_init - state_target) > 2e-1) and (mpc_iter * step_horizon < sim_time):
        t1 = time()

        args['p'] = ca.vertcat(
            state_init,    # current state
        )

        for i in range(1, N+1):
            p_target = p_state + v_ref * step_horizon * i
            p_idx = np.argmin(np.abs(path_p - p_target))
            args['p'] = ca.vertcat(args['p'],
                       ca.DM(path_mat[p_idx, :]))
            
        state_target = ca.DM(path_mat[p_idx, :])
        
        args['p'] = ca.vertcat(args['p'],
                       ca.DM(last_u,))
        
        u0 = ca.DM.zeros((n_controls, N))  # initial control
        X0 = ca.repmat(state_init, 1, N+1)         # initial state full

        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(X0, n_states*(N+1), 1),
            ca.reshape(u0, n_controls*N, 1)
        )

        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )
        if not solver.stats()["success"]:
            print("Infeasible Optimization")
            break


        u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
        X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)

        cat_states = np.dstack((
            cat_states,
            DM2Arr(X0)
        ))

        cat_controls = np.hstack((
            cat_controls,
            DM2Arr(u[:, 0])
        ))
        t = np.vstack((
            t,
            t0
        ))

        t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, motion_model, last_u)
        last_u = ca.vertcat(u0[0, 0], (1-alpha) * last_u[1] + alpha * u0[1, 0])

        closest_idx = np.argmin(np.linalg.norm(path_mat - state_init.T, axis=1))

        p_state = path_p[closest_idx]


        # print(X0)
        X0 = ca.horzcat(
            X0[:, 1:],
            ca.reshape(X0[:, -1], -1, 1)
        )

        # xx ...
        t2 = time()
        # print(t2-t1)
        times = np.vstack((
            times,
            t2-t1
        ))

        mpc_iter = mpc_iter + 1
        # if mpc_iter == 10:
        #     break


    main_loop_time = time()
    ss_error = ca.norm_2(state_init - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)

    plt.plot(cat_controls[0])
    plt.plot(cat_controls[1])


    # simulate
    simulate_path_tracking(cat_states, cat_controls, times, step_horizon, N, path_mat, save=False)
