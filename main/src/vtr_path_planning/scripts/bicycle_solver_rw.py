import sys
sys.dont_write_bytecode = True

import casadi as ca
from casadi import sin, cos, pi

# MPC for a model of a bicycle with tracking about the rear wheels
# Includes fixed first order lag

# distance from centre of gravity to front and rear wheels, respectively
# Based on the Hunter SE docs, for this formulation the actual centre of gravity
# is irrelevant since we track n the rear wheel
# TODO: Look into if its possible to make this configurable
l_f = 0.41
l_r = 0.41
L = l_r/l_f

step_horizon = 0.25  # time between steps in seconds
N = 15           # number of look ahead steps

# The first order lag weighting for the steering angle
alpha = 0.2

# state symbolic variables
# We assume psi is not a state, and model imperfect rates of change by including a first order lag, reducing the states
# from 4 to 3
# In cartesian coordinates considering the centre of the gravity for the vehicle, we have the following states
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(
    x,
    y,
    theta
)
n_states = states.numel()

rot_2d_z = ca.vertcat(
    ca.horzcat(cos(theta), -sin(theta)),
    ca.horzcat(sin(theta),  cos(theta))
    )

# control symbolic variables
v = ca.SX.sym('v')
psi = ca.SX.sym('psi')
controls = ca.vertcat(
    v,
    psi
)
n_controls = controls.numel()

# matrix containing all states over all time steps +1 (each column is a state vector)
X = ca.SX.sym('X', n_states, N + 1)

# matrix containing all control actions over all time steps (each column is an action vector)
U = ca.SX.sym('U', n_controls, N)

last_v = ca.SX.sym('last_v')
last_psi = ca.SX.sym('last_psi')
last_controls = ca.vertcat(
    last_v,
    last_psi
)

# number of parameters we can change without recompiling
num_parameters = 7
# column vector for storing runtime information(paths, etc) 
P = ca.SX.sym('P', n_states * (N+1) + n_controls + num_parameters)
measured_velo = P[-(n_controls+num_parameters):-num_parameters]
# TODO: Put this info into a config somewhere so this is easier to edit
Q_x = P[-num_parameters]
Q_y = P[-num_parameters + 1]
Q_theta = P[-num_parameters + 2]
R1 = P[-num_parameters + 4]
R2 = P[-num_parameters + 5]
Acc_R1 = P[-num_parameters + 7]
Acc_R2 = P[-num_parameters + 8]

# state weights matrix (Q_X, Q_Y, Q_THETA)
Q = ca.diagcat(Q_x, Q_y)

# controls weights matrix
R = ca.diagcat(R1, R2)

#Acceleration weith matrix
R_acc = ca.diagcat(Acc_R1, Acc_R2)

# Define kinematics of the systems
RHS = ca.vertcat(v*cos(theta), v*sin(theta), v/L * ca.tan(alpha*last_psi + (1-alpha)*psi))
motion_model = ca.Function('motion_model', [states, controls, last_controls], [RHS])

theta_to_so2 = ca.Function('theta2rotm', [theta], [rot_2d_z])

cost_fn = 0  # cost function
g = X[:, 0] - P[:n_states]  # constraints in the equation


def so2_error(ref, current):
    rel_m = theta_to_so2(ref).T @ theta_to_so2(current)
    return ca.atan2(rel_m[1, 0], rel_m[0, 0])

#for initial
k = 0
st = X[:, k]
con = U[:, k]
last_vel = measured_velo
st_next = X[:, k+1]
cost_fn = cost_fn \
        + (st_next[:2] - P[n_states*(k+1):n_states*(k+2)-1]).T @ Q @ (st_next[:2] - P[n_states*(k+1):n_states*(k+2)-1]) \
        + con.T @ R @ con \
        + so2_error(P[n_states*(k+1) + 2], st_next[2]) * Q_theta * so2_error(P[n_states*(k+1) + 2], st_next[2])
k1 = motion_model(st, con, last_vel)
k2 = motion_model(st + step_horizon/2*k1, con, last_vel)
k3 = motion_model(st + step_horizon/2*k2, con, last_vel)
k4 = motion_model(st + step_horizon * k3, con, last_vel)
st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
# st_next_int = motion_model(st, con)
g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))

def state_cost(st_next, P_next):

    pass

def control_cost(u_k, u_last):
    pass

# runge kutta
for k in range(1, N):
    st = X[:, k]
    st_next = X[:, k+1]

    con = U[:, k]
    last_vel = ca.vertcat(U[0, k-1], (1 - alpha) * U[1, k-1] + alpha * last_vel[1])
    cost_fn = cost_fn \
        + (st_next[:2] - P[n_states*(k+1):n_states*(k+2)-1]).T @ Q @ (st_next[:2] - P[n_states*(k+1):n_states*(k+2)-1]) \
        + con.T @ R @ con \
        + so2_error(P[n_states*(k+1) + 2], st_next[2]) * Q_theta * so2_error(P[n_states*(k+1) + 2], st_next[2])

    k1 = motion_model(st, con, last_vel)
    k2 = motion_model(st + step_horizon/2*k1, con, last_vel)
    k3 = motion_model(st + step_horizon/2*k2, con, last_vel)
    k4 = motion_model(st + step_horizon * k3, con, last_vel)
    st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    # st_next_int = motion_model(st, con)

    g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
    g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))


for k in range(N):
    theta_k = P[n_states*(k+1) + 2]
    g = ca.vertcat(g, ca.vertcat(-sin(theta_k), cos(theta_k)).T @ (X[:2, k] - P[n_states*(k+1): n_states*(k+1)+2]))

#Acceleration constraints
cost_fn += (U[:, 0] - measured_velo).T @ R_acc @ (U[:, 0] - measured_velo)
for k in range(1, N):
    cost_fn += (U[:, k] - U[:, k-1]).T @ R_acc @ (U[:, k] - U[:, k-1])


OPT_variables = ca.vertcat(
    X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
    U.reshape((-1, 1))
)
nlp_prob = {
    'f': cost_fn,
    'x': OPT_variables,
    'g': g,
    'p': P
}

opts = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'acceptable_tol': 1e-5,
        'acceptable_obj_change_tol': 1e-4
    },
    'print_time': 0
}

solver = ca.nlpsol('solve_bicycle_mpc', 'ipopt', nlp_prob, opts)
