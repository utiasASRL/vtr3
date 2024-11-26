# This controller is for an ackermann drive robot that receives linear and agular velocity twist commands.
# There is a turning radius constraint that helps the robot drive forward. 
# There is a higher cost to driving backwards than forwards

import sys
sys.dont_write_bytecode = True

import casadi as ca
from casadi import sin, cos, pi

#Compile Time Constants (Could use params to set!)


# Pose Covariance
Q_x = 1
Q_y = 1
Q_theta = 1

# Command Covariance
R1_fwd = 1.0
R1_rev = 100.0
R2 = 10.0
# turning_r = 20

# Acceleration Cost Covariance
Acc_R1 = 0.1
Acc_R2 = 0.5

step_horizon = 0.25  # time between steps in seconds
N = 15           # number of look ahead steps

# The first order lag weighting for the angular velocity
alpha = 0.2

# state symbolic variables
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
omega = ca.SX.sym('omega')
controls = ca.vertcat(
    v,
    omega
)
n_controls = controls.numel()

# matrix containing all states over all time steps +1 (each column is a state vector)
X = ca.SX.sym('X', n_states, N + 1)

# matrix containing all control actions over all time steps (each column is an action vector)
U = ca.SX.sym('U', n_controls, N)

last_v = ca.SX.sym('last_v')
last_omega = ca.SX.sym('last_omega')
last_controls = ca.vertcat(
    last_v,
    last_omega
)

# column vector for storing initial state and target states + initial velocity
P = ca.SX.sym('P', n_states * (N+1) + n_controls + 1)
measured_velo = P[-3:-1]
turning_r = P[-1]

# state weights matrix (Q_X, Q_Y, Q_THETA)
Q = ca.diagcat(Q_x, Q_y)

# controls weights matrix
R = ca.diagcat(R1_fwd, R2)

#Acceleration weith matrix
R_acc = ca.diagcat(Acc_R1, Acc_R2)


RHS = ca.vertcat(v*cos(theta), v*sin(theta), last_omega*alpha + (1-alpha)*omega)
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

    g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
    g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))


for k in range(N):
    theta_k = P[n_states*(k+1) + 2]
    g = ca.vertcat(g, ca.vertcat(-sin(theta_k), cos(theta_k)).T @ (X[:2, k] - P[n_states*(k+1): n_states*(k+1)+2]))

for k in range(N):
    g = ca.vertcat(g, U[0, k] / turning_r + U[1, k])
    g = ca.vertcat(g, -U[0, k] / turning_r + U[1, k])


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

solver = ca.nlpsol('solve_unicycle_mpc', 'ipopt', nlp_prob, opts)
