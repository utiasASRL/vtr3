from time import time
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from simulation_code import simulate_path_tracking_convoy

from unicycle_solver import solver, motion_model, alpha, N, step_horizon, n_states, n_controls
from unicycle_follower_solver import solver as solver_follower


# specs
p_init_l = 5.45
p_init_f = 0

dist = 5.0
dist_margin = 100.0

v_max = 1.5
v_min = -1.5
w_max = 0.5
w_min = -0.5
v_ref = 1.0

lin_acc_max = 1.00
ang_acc_max = 0.5


sim_time = 100      # simulation time

euclidean_distance = True



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
#path_y[-1000:] = 0.5*path_x[-1000:]
path_mat = np.zeros((np.size(path_x), 3))
path_mat[:, 0] = path_x
path_mat[:, 1] = path_y
path_mat[:, 2] = np.arctan2(np.gradient(path_y), np.gradient(path_x))
path_p = np.zeros_like(path_x)
for i in range(1, np.size(path_p)):
    path_p[i] = path_p[i-1] + np.hypot(path_x[i] - path_x[i-1], path_y[i] - path_y[i-1]) + 0.25*np.abs(path_mat[i, 2] - path_mat[i-1, 2])


# Find index of path_p where path_p is closest to p_init_l
closest_idx_l = np.argmin(np.abs(path_p - p_init_l))
closest_idx_f = np.argmin(np.abs(path_p - p_init_f))

# Set initial values accordingly
x_init_l = path_mat[closest_idx_l, 0]
y_init_l = path_mat[closest_idx_l, 1]
theta_init_l = path_mat[closest_idx_l, 2]

x_init_f = path_mat[closest_idx_f, 0]
y_init_f = path_mat[closest_idx_f, 1]
theta_init_f = path_mat[closest_idx_f, 2]

# Set init values in task space
state_init_l = ca.DM([x_init_l, y_init_l, theta_init_l]) 
state_init_f = ca.DM([x_init_f, y_init_f, theta_init_f])   

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
lbg_l = ca.DM.zeros((n_states*(N+1) + N, 1))  # constraints lower bound
ubg_l = ca.DM.zeros((n_states*(N+1) + N, 1))  # constraints upper bound
lbg_f = ca.DM.zeros((n_states*(N+1) + 2*N, 1))  # constraints lower bound
ubg_f = ca.DM.zeros((n_states*(N+1) + 2*N, 1))  # constraints upper bound

#Corridor Width constraints
lbg_l[n_states*(N+1):n_states*(N+1)+N] = -100
ubg_l[n_states*(N+1):n_states*(N+1)+N] = 100
lbg_f[n_states*(N+1):n_states*(N+1)+N] = -100
ubg_f[n_states*(N+1):n_states*(N+1)+N] = 100

lbg_f[n_states*(N+1)+N:n_states*(N+1)+2*N] = dist - dist_margin
ubg_f[n_states*(N+1)+N:n_states*(N+1)+2*N] = dist + dist_margin


#Acceleration constraints
# lbg[n_states*(N+1)+N::2] = -lin_acc_max * step_horizon
# ubg[n_states*(N+1)+N::2] = lin_acc_max * step_horizon
# lbg[n_states*(N+1)+N+1::2] = -ang_acc_max * step_horizon
# ubg[n_states*(N+1)+N+1::2] = ang_acc_max * step_horizon

print(lbx.shape)

args_l = {
    'lbg': lbg_l,
    'ubg': ubg_l,  
    'lbx': lbx,
    'ubx': ubx
}

args_f = {
    'lbg': lbg_f,
    'ubg': ubg_f,  
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
    p_state_l = p_init_l
    p_state_f = p_init_f
    
    
    last_leader_rollout = None
    last_leader_vel_rollout = None
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
            if last_leader_vel_rollout is None:
                p_target_f = p_state_f + last_u_l[1] * step_horizon * i
                p_idx_f = np.argmin(np.abs(path_p - p_target_f))
            else:
                if i < N-2:
                    leader_position = np.array([float(last_leader_rollout[0, i+1]), float(last_leader_rollout[1, i+1])])
                else:
                    leader_position = np.array([float(last_leader_rollout[0, N]), float(last_leader_rollout[1, N])])

                distances_to_leader = np.linalg.norm(path_mat[:, :2] - leader_position, axis=1)

                if euclidean_distance == False:
                    # Get closest point on reference path to last_leader_rollout[0, i+1], last_leader_rollout[1, i+1]
                    closest_point = np.argmin(distances_to_leader)
                    p_target_f = path_p[closest_point] - dist
                    p_idx_f = np.argmin(np.abs(path_p - p_target_f))
                else: # Find point on path that is dist away from leader (last_leader_rollout[0, i+1], last_leader_rollout[1, i+1]) in euclidean space
                    closest_point_l = np.argmin(distances_to_leader)
                    closest_point_f = np.argmin(np.abs(distances_to_leader[:closest_point_l] - dist))
                    p_idx_f = closest_point_f
            
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
                if i < N-2:
                    args_f['p'] = ca.vertcat(args_f['p'],
                            ca.DM([last_leader_rollout[0, i+1], last_leader_rollout[1, i+1], last_leader_rollout[2, i+1]]))
                else:
                    args_f['p'] = ca.vertcat(args_f['p'],
                            ca.DM([last_leader_rollout[0, N], last_leader_rollout[1, N], last_leader_rollout[2, N]]))
        
        #if last_leader_rollout is not None:
        #    args_f['p'] = ca.vertcat(args_f['p'],
        #        ca.DM([last_leader_rollout[0, 0], last_leader_rollout[1, 0], last_leader_rollout[2, 0]]))
        
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
        last_leader_vel_rollout = u_l

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

    # Plot distance over time between cat_states_l and cat_states_f on the path (distance on arclength not euclidean distance)
    arc_distances = []
    for i in range(cat_states_l.shape[2]):
        leader_state = cat_states_l[:2, 0, i]
        follower_state = cat_states_f[:2, 0, i]
        closest_point_leader = np.argmin(np.linalg.norm(path_mat[:, :2] - leader_state.T, axis=1))
        closest_point_follower = np.argmin(np.linalg.norm(path_mat[:, :2] - follower_state.T, axis=1))
        arc_distance = np.abs(path_p[closest_point_leader] - path_p[closest_point_follower])
        arc_distances.append(arc_distance)

    plt.figure()
    plt.plot(arc_distances)
    plt.xlabel('Time step')
    plt.ylabel('Arc Distance between leader and follower')
    plt.title('Arc Distance over time between leader and follower')

    # plot path tracking error for each robot
    plt.figure()
    plt.plot(cat_states_l[0, 0, :], cat_states_l[1, 0, :], label='Leader')
    plt.plot(cat_states_f[0, 0, :], cat_states_f[1, 0, :], label='Follower')
    plt.plot(path_x, path_y, label='Reference Path', linestyle='--')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Path Tracking')
    plt.legend()

    # Compute and plot signed path tracking error for each robot
    leader_errors = []
    follower_errors = []
    for i in range(cat_states_l.shape[2]):
        leader_state = cat_states_l[:2, 0, i]
        follower_state = cat_states_f[:2, 0, i]
        closest_point_leader = np.argmin(np.linalg.norm(path_mat[:, :2] - leader_state.T, axis=1))
        closest_point_follower = np.argmin(np.linalg.norm(path_mat[:, :2] - follower_state.T, axis=1))
        
        leader_error_vector = path_mat[closest_point_leader, :2] - leader_state.T
        follower_error_vector = path_mat[closest_point_follower, :2] - follower_state.T
        
        leader_error_sign = np.sign(np.cross(path_mat[closest_point_leader, :2], leader_error_vector))
        follower_error_sign = np.sign(np.cross(path_mat[closest_point_follower, :2], follower_error_vector))
        
        leader_errors.append(leader_error_sign * np.linalg.norm(leader_error_vector))
        follower_errors.append(follower_error_sign * np.linalg.norm(follower_error_vector))

    plt.figure()
    plt.plot(leader_errors, label='Leader Error')
    plt.plot(follower_errors, label='Follower Error')
    plt.xlabel('Time step')
    plt.ylabel('Signed Tracking Error')
    plt.title('Signed Path Tracking Error')
    plt.legend()
    plt.show()


    # simulate
    simulate_path_tracking_convoy(cat_states_l, cat_controls_l, cat_states_f, cat_controls_f, times, step_horizon, N, path_mat, save=False)