import sys
sys.dont_write_bytecode = True

import casadi as ca
from casadi import sin, cos, pi, tan

# MPC for a model of a bicycle with tracking about the rear wheels
# Includes fixed first order lag


step_horizon = 0.25  # time between steps in seconds
N = 15           # number of look ahead steps

# The first order lag weighting for the steering angle
alpha = 0.6

alpha_v = 0.0

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

# column vector for storing runtime information(paths, etc) 
init_pose = ca.SX.sym('init_pose', n_states)
ref_poses = ca.SX.sym('ref_poses_l', n_states*N)
measured_velo = ca.SX.sym('measured_velo', n_controls)
cost_weights = ca.SX.sym('cost_weighting', N)  # weight for the cost function at each timestep

Q_lat = ca.SX.sym('Q_lat', 1)
Q_lon = ca.SX.sym('Q_lon', 1)
Q_theta = ca.SX.sym('Q_theta', 1)
R1 = ca.SX.sym('R1', 1)
R2 = ca.SX.sym('R2', 1)
Acc_R1 = ca.SX.sym('Acc_R1', 1)
Acc_R2 = ca.SX.sym('Acc_R2', 1)
Q_f = ca.SX.sym('Q_f', 1)  # final state cost
L = ca.SX.sym('wheel_base', 1)

P = ca.vertcat(init_pose, ref_poses, measured_velo, cost_weights,               # Base MPC
                L, Q_lat, Q_lon, Q_theta, R1, R2, Acc_R1 , Acc_R2, Q_f)    # Weights for tuning



# state weights matrix (Q_X, Q_Y, Q_THETA)
Q = ca.diagcat(Q_lat, Q_lon)

# controls weights matrix
R = ca.diagcat(R1, R2)

#Acceleration weith matrix
R_acc = ca.diagcat(Acc_R1, Acc_R2)

# Define kinematics of the systems
weighted_v = alpha_v*last_v + (1-alpha_v)*v
weighted_psi = alpha*last_psi + (1-alpha)*psi
RHS = ca.vertcat(weighted_v*cos(theta), weighted_v*sin(theta), weighted_v/L * tan(weighted_psi))
motion_model = ca.Function('motion_model', [states, controls, last_controls, L], [RHS])

theta_to_so2 = ca.Function('theta2rotm', [theta], [rot_2d_z])

cost_fn = 0  # cost function
g = X[:, 0] - P[:n_states]  # constraints in the equation


def so2_error(ref, current):
    rel_m = theta_to_so2(ref).T @ theta_to_so2(current)
    return ca.atan2(rel_m[1, 0], rel_m[0, 0])

def calc_cost(ref, X, con, k, cost_weight):
    dx = X[0, k+1] - ref[n_states*(k+1)]
    dy = X[1, k+1] - ref[n_states*(k+1)+1]
    theta_ref = ref[n_states*(k+1)+2]
    e_lat = -sin(theta_ref)*dx + cos(theta_ref)*dy
    e_lon = cos(theta_ref)*dx + sin(theta_ref)*dy
    cost = cost_weight*(Q_lat * e_lat**2 + Q_lon * e_lon**2 + Q_theta*so2_error(theta_ref, X[2,k+1])**2 + con.T @ R @ con)
    return cost

#for initial
k = 0
st = X[:, k]
con = U[:, k]
last_vel = measured_velo
st_next = X[:, k+1]
cost_fn += calc_cost(P, X, con, k, cost_weights[k])
k1 = motion_model(st, con, last_vel, L)
k2 = motion_model(st + step_horizon/2*k1, con, last_vel, L)
k3 = motion_model(st + step_horizon/2*k2, con, last_vel, L)
k4 = motion_model(st + step_horizon * k3, con, last_vel, L)
st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))

# runge kutta
for k in range(1, N):
    st = X[:, k]
    st_next = X[:, k+1]

    con = U[:, k]
    last_vel = ca.vertcat(U[0, k-1], (1 - alpha) * U[1, k-1] + alpha * last_vel[1])

    cost_fn += calc_cost(P, X,con, k, cost_weights[k])
    k1 = motion_model(st, con, last_vel, L)
    k2 = motion_model(st + step_horizon/2*k1, con, last_vel, L)
    k3 = motion_model(st + step_horizon/2*k2, con, last_vel, L)
    k4 = motion_model(st + step_horizon * k3, con, last_vel, L)
    st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
    g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))


for k in range(N):
    theta_k = P[n_states*(k+1) + 2]
    g = ca.vertcat(g, ca.vertcat(-sin(theta_k), cos(theta_k)).T @ (X[:2, k] - P[n_states*(k+1): n_states*(k+1)+2]))

#Acceleration constraints
cost_fn += cost_weights[0]*((U[:, 0] - measured_velo).T @ R_acc @ (U[:, 0] - measured_velo))
g = ca.vertcat(g, U[0, 0])
g = ca.vertcat(g, U[1, 0])
for k in range(1, N-1):
    cost_fn += cost_weights[k]*((U[:, k] - U[:, k-1]).T @ R_acc @ (U[:, k] - U[:, k-1]))
    #cost_fn += 0.1/(U[0, k]**2 + 1e-3)
    # Add acceleration constraints
    g = ca.vertcat(g, U[0, k] - U[0, k-1])
    # Angular acceleration constraints
    g = ca.vertcat(g, U[1, k] - U[1, k-1])

# Terminal cost
cost_fn += calc_cost(P, X, con, N-1, cost_weights[N-1])

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
