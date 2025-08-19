from time import time
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

from bicycle_follower_joint_solver_pt import solver, alpha, N, step_horizon, n_states, n_controls


# specs
x_init_l = 4.5
y_init_l = 0
theta_init_l = 0

x_init_f = 0
y_init_f = 0
theta_init_f = 0

dist = 5

v_max = 1.5
v_min = -1.5
theta_max = 0.5
theta_min = -0.5
v_ref = 0.75*v_max

lin_acc_max = 1.5
ang_vel_max = 0.5

dist_margin = 10


sim_time = 100      # simulation time

state_init_l = ca.DM([x_init_l, y_init_l, theta_init_l]) 
state_init_f = ca.DM([x_init_f, y_init_f, theta_init_f])   

lbx = ca.DM.zeros((2*n_states*(N+1) + 2*n_controls*N, 1))
ubx = ca.DM.zeros((2*n_states*(N+1) + 2*n_controls*N, 1))

lbx[0: 2*n_states*(N+1): n_states] = -ca.inf     # X lower bound
lbx[1: 2*n_states*(N+1): n_states] = -ca.inf     # Y lower bound
lbx[2: 2*n_states*(N+1): n_states] = -ca.inf     # theta lower bound

ubx[0: 2*n_states*(N+1): n_states] = ca.inf      # X upper bound
ubx[1: 2*n_states*(N+1): n_states] = ca.inf      # Y upper bound
ubx[2: 2*n_states*(N+1): n_states] = ca.inf      # theta upper bound

lbx[2*n_states*(N+1)::2] = v_min                  # v lower bound for all V
ubx[2*n_states*(N+1)::2] = v_max                  # v upper bound for all V
lbx[2*n_states*(N+1)+1::2] = theta_min            # v upper bound for all V
ubx[2*n_states*(N+1)+1::2] = theta_max            # v upper bound for all V

#Constraints
lbg = ca.DM.zeros((2*n_states*(N+1) + 2*(N-1)*n_controls + N, 1))  # constraints lower bound
ubg = ca.DM.zeros((2*n_states*(N+1) + 2*(N-1)*n_controls + N, 1))  # constraints upper bound

#Acceleration constraints
lbg[2*n_states*(N+1):2*n_states*(N+1) + 2*(N-1)*n_controls:2] = -lin_acc_max * step_horizon
ubg[2*n_states*(N+1):2*n_states*(N+1) + 2*(N-1)*n_controls:2] = lin_acc_max * step_horizon

lbg[2*n_states*(N+1) + 1:2*n_states*(N+1) + 2*(N-1)*n_controls:2] = -ang_vel_max * step_horizon
ubg[2*n_states*(N+1) + 1:2*n_states*(N+1) + 2*(N-1)*n_controls:2] = ang_vel_max * step_horizon

#First states (The same assuming 0,0)
# lbg[2*n_states*(N+1):2*n_states*(N+1) + 2*(N-1)*n_controls:2] = -lin_acc_max * step_horizon
# ubg[2*n_states*(N+1):2*n_states*(N+1) + 2*(N-1)*n_controls:2] = lin_acc_max * step_horizon

#Corridor Width constraints
lbg[2*n_states*(N+1) + 2*(N-1)*n_controls:] = dist - dist_margin
ubg[2*n_states*(N+1) + 2*(N-1)*n_controls:] = dist + dist_margin

print(ubg)

# Leader initial state
p = ca.DM([x_init_l, y_init_l, theta_init_l]) 
for x_l in range(1, N+1):
    p = ca.vertcat(p, ca.DM([x_init_l + v_ref*x_l*step_horizon, 0, 0]))
p = ca.vertcat(p, ca.DM([0, 0]))

# Follower initial state
p = ca.vertcat(p, ca.DM([x_init_f, y_init_f, theta_init_f]))
for x_f in range(1, N+1):
    p = ca.vertcat(p, ca.DM([x_init_f + v_ref*x_f*step_horizon, 0, 0]))
p = ca.vertcat(p, ca.DM([0, 0]))
p = ca.vertcat(p, ca.DM(dist))

print(p)

#Params
p = ca.vertcat(p, ca.DM(0.65)) # wheelbase
p = ca.vertcat(p, ca.DM(10)) # Q_lat
p = ca.vertcat(p, ca.DM(4)) # Q_lon
p = ca.vertcat(p, ca.DM(7)) # Q_theta
p = ca.vertcat(p, ca.DM(0)) # R1
p = ca.vertcat(p, ca.DM(3)) # R2
p = ca.vertcat(p, ca.DM(4)) # Acc_R1
p = ca.vertcat(p, ca.DM(5)) # Acc_R2
p = ca.vertcat(p, ca.DM(1)) # Q_f
p = ca.vertcat(p, ca.DM(100)) # Q_dist

u0 = ca.DM.zeros((2*n_controls, N))  # initial control
X0 = ca.repmat(ca.vertcat(state_init_l, state_init_f), 1, N+1)         # initial state full

# optimization variable current state
x0 = ca.vertcat(
    ca.reshape(X0, 2*n_states*(N+1), 1),
    ca.reshape(u0, 2*n_controls*N, 1)
)

joint_sol = solver(
            x0=x0,
            lbx=lbx,
            ubx=ubx,
            lbg=lbg,
            ubg=ubg,
            p=p
        )
if not solver.stats()["success"]:
            print(solver.stats()['return_status'])
            print(solver.stats())

u = ca.reshape(joint_sol['x'][2 * n_states * (N + 1):], 2*n_controls, N)

X0 = ca.reshape(joint_sol['x'][:2 * n_states * (N+1)], 2*n_states, N+1)
X0_l = X0[:n_states, :]
X0_f = X0[n_states:, :]
print(X0_l[0, :])
print(X0_f[0, :])