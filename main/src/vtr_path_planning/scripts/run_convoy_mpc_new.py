from time import time
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from simulation_code import simulate_path_tracking_convoy

from unicycle_solver import solver, motion_model, alpha, N, step_horizon, n_states, n_controls
from unicycle_follower_solver import solver as solver_follower


# specs
x_init_l = 0
y_init_l = 0
theta_init_l = 0

x_init_f = 0
y_init_f = 0
theta_init_f = 0

dist = 5

v_max = 1.5
v_min = -1.5
w_max = 0.5
w_min = -0.5
v_ref = 0.5*v_max

lin_acc_max = 1.00
ang_acc_max = 0.5


sim_time = 100      # simulation time


state_init_l = ca.DM([x_init_l, y_init_l, theta_init_l])        # initial state

state_init_f = ca.DM([x_init_f, y_init_f, theta_init_f])        # initial state

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

path_x = np.linspace(0, 50, 10000)
path_y = 0.5*path_x + np.sin(2*np.pi*path_x/10)
path_y[-1000:] = 0.5*path_x[-1000:]
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
lbg = ca.DM.zeros((n_states*(N+1) + N, 1))  # constraints lower bound
ubg = ca.DM.zeros((n_states*(N+1) + N, 1))  # constraints upper bound

#Corridor Width constraints
lbg[n_states*(N+1):n_states*(N+1)+N] = -5
ubg[n_states*(N+1):n_states*(N+1)+N] = 5

#Acceleration constraints
# lbg[n_states*(N+1)+N::2] = -lin_acc_max * step_horizon
# ubg[n_states*(N+1)+N::2] = lin_acc_max * step_horizon
# lbg[n_states*(N+1)+N+1::2] = -ang_acc_max * step_horizon
# ubg[n_states*(N+1)+N+1::2] = ang_acc_max * step_horizon

print(lbx.shape)

args_l = {
    'lbg': lbg,
    'ubg': ubg,  
    'lbx': lbx,
    'ubx': ubx
}

args_f = {
    'lbg': lbg,
    'ubg': ubg,  
    'lbx': lbx,
    'ubx': ubx
}

t0 = 0

t = ca.DM(t0)

u0_l = 0.1*ca.DM.ones((n_controls, N))  # initial control
X0_l = ca.repmat(state_init_l, 1, N+1)         # initial state full

u0_f = 0.1*ca.DM.ones((n_controls, N))  # initial control
X0_f = ca.repmat(state_init_f, 1, N+1)         # initial state full


mpc_iter = 0

cat_states_l = DM2Arr(X0_l)
cat_controls_l = DM2Arr(u0_l[:, 0])
last_u_l = u0_l[:, 0]

cat_states_f = DM2Arr(X0_f)
cat_controls_f = DM2Arr(u0_f[:, 0])
last_u_f = u0_f[:, 0]

times = np.array([[0]])
state_target = path_mat[-1, :]

###############################################################################

if __name__ == '__main__':
    main_loop = time()  # return time in sec
    p_state_l = 0
    p_state_f = 0
    
    
    last_leader_rollout = None
    u_l = None
    while (ca.norm_2(state_init_l - state_target) > 2e-1) and (mpc_iter * step_horizon < sim_time):
        t1 = time()

        # leader init pose
        args_l['p'] = ca.vertcat(
            state_init_l,    # current state
        )

        # follower init pose
        args_f['p'] = ca.vertcat(
            state_init_f,    # current state
        )

        # follower waypoints
        for i in range(1, N+1):
            if last_leader_rollout is None:
                p_target_f = p_state_f + last_u_l[1] * step_horizon * i
            else:
                p_target_f = p_state_f + u_l[0,1] * step_horizon * i
            p_idx_f = np.argmin(np.abs(path_p - p_target_f))
            args_f['p'] = ca.vertcat(args_f['p'],
                       ca.DM(path_mat[p_idx_f, :]))
        
        # follower init speed
        args_f['p'] = ca.vertcat(args_f['p'],
                       ca.DM(last_u_f,))

        # leader waypoints
        for i in range(1, N+1):
            p_target_l = p_state_l + v_ref * step_horizon * i
            p_idx_l = np.argmin(np.abs(path_p - p_target_l))
            args_l['p'] = ca.vertcat(args_l['p'],
                       ca.DM(path_mat[p_idx_l, :]))
            if last_leader_rollout is None:
                args_f['p'] = ca.vertcat(args_f['p'],
                        ca.DM(path_mat[p_idx_l, :]))
            else:
                if i < N:
                    args_f['p'] = ca.vertcat(args_f['p'],
                            ca.DM([last_leader_rollout[0, i+1], last_leader_rollout[1, i+1], last_leader_rollout[2, i+1]]))
        
        if last_leader_rollout is not None:
            args_f['p'] = ca.vertcat(args_f['p'],
                ca.DM([last_leader_rollout[0, 0], last_leader_rollout[1, 0], last_leader_rollout[2, 0]]))
        
        state_target = ca.DM(path_mat[p_idx_l, :])


        # leader init speed
        args_l['p'] = ca.vertcat(args_l['p'],
                       ca.DM(last_u_l,))

        # desired distance
        args_f['p'] = ca.vertcat(args_f['p'],
                       ca.DM(dist,))
        
        
        u0_l = ca.DM.zeros((n_controls, N))  # initial control
        X0_l = ca.repmat(state_init_l, 1, N+1)         # initial state full


        u0_f = ca.DM.zeros((n_controls, N))  # initial control
        X0_f = ca.repmat(state_init_f, 1, N+1)         # initial state full

        # optimization variable current state
        args_l['x0'] = ca.vertcat(
            ca.reshape(X0_l, n_states*(N+1), 1),
            ca.reshape(u0_l, n_controls*N, 1)
        )

        # optimization variable current state
        args_f['x0'] = ca.vertcat(
            ca.reshape(X0_f, n_states*(N+1), 1),
            ca.reshape(u0_f, n_controls*N, 1)
        )

        sol_l = solver(
            x0=args_l['x0'],
            lbx=args_l['lbx'],
            ubx=args_l['ubx'],
            lbg=args_l['lbg'],
            ubg=args_l['ubg'],
            p=args_l['p']
        )
        if not solver.stats()["success"]:
            print(solver.stats()['return_status'])
            print(solver.stats())
            break

        

        sol_f = solver_follower(
            x0=args_f['x0'],
            lbx=args_f['lbx'],
            ubx=args_f['ubx'],
            lbg=args_f['lbg'],
            ubg=args_f['ubg'],
            p=args_f['p']
        )
        if not solver_follower.stats()["success"]:
            print(solver_follower.stats()['return_status'])
            print(solver_follower.stats())
            break


        u_l = ca.reshape(sol_l['x'][n_states * (N + 1):], n_controls, N)
        u_f = ca.reshape(sol_f['x'][n_states * (N + 1):], n_controls, N)
        # print(u)
        
        X0_l = ca.reshape(sol_l['x'][: n_states * (N+1)], n_states, N+1)
        X0_f = ca.reshape(sol_f['x'][: n_states * (N+1)], n_states, N+1)

        last_leader_rollout = X0_l

        cat_states_l = np.dstack((
            cat_states_l,
            DM2Arr(X0_l)
        ))

        cat_states_f = np.dstack((
            cat_states_f,
            DM2Arr(X0_f)
        ))

        cat_controls_l = np.hstack((
            cat_controls_l,
            DM2Arr(u_l[:, 0])
        ))

        cat_controls_f = np.hstack((
            cat_controls_f,
            DM2Arr(u_f[:, 0])
        ))
        t = np.vstack((
            t,
            t0
        ))

        t_discard, state_init_l, u0_l = shift_timestep(step_horizon, t0, state_init_l, u_l, motion_model, last_u_l)
        t0, state_init_f, u0_f = shift_timestep(step_horizon, t0, state_init_f, u_f, motion_model, last_u_f)

        last_u_l = ca.vertcat(u_l[0, 0], (1-alpha) * last_u_l[1] + alpha * u_l[1, 0])
        last_u_f = ca.vertcat(u_f[0, 0], (1-alpha) * last_u_f[1] + alpha * u_f[1, 0])

        closest_idx_l = np.argmin(np.linalg.norm(path_mat - state_init_l.T, axis=1))
        closest_idx_f = np.argmin(np.linalg.norm(path_mat - state_init_f.T, axis=1))

        p_state_l = path_p[closest_idx_l]
        p_state_f = path_p[closest_idx_f]


        # print(X0)
        X0_l = ca.horzcat(
            X0_l[:, 1:],
            ca.reshape(X0_l[:, -1], -1, 1)
        )
        X0_f = ca.horzcat(
            X0_f[:, 1:],
            ca.reshape(X0_f[:, -1], -1, 1)
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
    ss_error = ca.norm_2(state_init_l - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)

    plt.plot(cat_controls_l[0])
    plt.plot(cat_controls_l[1])
    plt.plot(cat_controls_f[0])
    plt.plot(cat_controls_f[1])

    # plot distance over time between cat_states_l and cat_states_f
    distances = np.linalg.norm(cat_states_l[:2, 0, :] - cat_states_f[:2, 0, :], axis=0)
    plt.figure()
    plt.plot(distances.T)
    plt.xlabel('Time step')
    plt.ylabel('Distance between leader and follower')
    plt.title('Distance over time between leader and follower')


    # simulate
    simulate_path_tracking_convoy(cat_states_l, cat_controls_l, cat_states_f, cat_controls_f, times, step_horizon, N, path_mat, save=True)