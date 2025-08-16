import sys
sys.dont_write_bytecode = True

import casadi as ca
from casadi import sin, cos, pi, tan

# MPC for a model of a bicycle with tracking about the rear wheels
# Following a lead vehicle.
# Includes fixed first order lag


step_horizon = 0.25  # time between steps in seconds
N = 15          # number of look ahead steps

# The first order lag weighting for the steering angle
alpha = 0.7

alphav = 0.0

# state symbolic variables
# We assume psi is not a state, and model imperfect rates of change by including a first order lag, reducing the states
# from 4 to 3
# In cartesian coordinates considering the centre of the gravity for the vehicle, we have the following states
x_l = ca.SX.sym('x_l')
y_l = ca.SX.sym('y_l')
theta_l = ca.SX.sym('theta_l')
leader_states = ca.vertcat(
    x_l,
    y_l,
    theta_l
)

x_f = ca.SX.sym('x_f')
y_f = ca.SX.sym('y_f')
theta_f = ca.SX.sym('theta_f')
follower_states = ca.vertcat(
    x_f,
    y_f,
    theta_f
)

states = ca.vertcat(leader_states, follower_states)

n_states = leader_states.numel()

rot_2d_z = ca.vertcat(
    ca.horzcat(cos(theta_l), -sin(theta_l)),
    ca.horzcat(sin(theta_l),  cos(theta_l))
    )

# control symbolic variables
v_l = ca.SX.sym('v_l')
psi_l = ca.SX.sym('psi_l')
leader_controls = ca.vertcat(
    v_l,
    psi_l
)
v_f = ca.SX.sym('v_f')
psi_f = ca.SX.sym('psi_f')
follower_controls = ca.vertcat(
    v_f,
    psi_f
)

controls = ca.vertcat(leader_controls, follower_controls)
n_controls = leader_controls.numel()

# matrix containing all states over all time steps +1 (each column is a state vector)
X = ca.SX.sym('X', 2*n_states, N + 1)

# matrix containing all control actions over all time steps (each column is an action vector)
U = ca.SX.sym('U', 2*n_controls, N)

last_v_l = ca.SX.sym('last_v_l')
last_psi_l = ca.SX.sym('last_psi_l')
last_controls_l = ca.vertcat(
    last_v_l,
    last_psi_l
)

last_v_f = ca.SX.sym('last_v_f')
last_psi_f = ca.SX.sym('last_psi_f')
last_controls_f = ca.vertcat(
    last_v_f,
    last_psi_f
)



# column vector for storing runtime information(paths, etc) 
init_pose_leader = ca.SX.sym('init_pose_l', n_states)
init_pose_follower = ca.SX.sym('init_pose_f', n_states)
follower_ref_poses = ca.SX.sym('ref_poses_f', n_states*N)
leader_ref_poses = ca.SX.sym('ref_poses_l', n_states*N)
measured_velo_leader = ca.SX.sym('measured_velo_l', n_controls)
measured_velo_follower = ca.SX.sym('measured_velo_f', n_controls)

Q_lat = ca.SX.sym('Q_lat', 1)
Q_lon = ca.SX.sym('Q_lat', 1)
Q_theta = ca.SX.sym('Q_theta', 1)
R1 = ca.SX.sym('R1', 1)
R2 = ca.SX.sym('R2', 1)
Acc_R1 = ca.SX.sym('Acc_R1', 1)
Acc_R2 = ca.SX.sym('Acc_R2', 1)
Q_f = ca.SX.sym('Q_f', 1)  # final state cost
d = ca.SX.sym('d', 1)
Q_dist = ca.SX.sym('Q_dist', 1)
L = ca.SX.sym('wheel_base', 1)

P = ca.vertcat(init_pose_leader, leader_ref_poses, measured_velo_leader,                # Base MPC
                init_pose_follower, follower_ref_poses, measured_velo_follower, d,                                      # Follower specific
                L, Q_lat, Q_lon, Q_theta, R1, R2, Acc_R1 , Acc_R2, Q_f, Q_dist)    # Weights for tuning


# state weights matrix (Q_lat, Q_lon)
Q = ca.diagcat(Q_lat, Q_lon)

# controls weights matrix
R = ca.diagcat(R1, R2)

#Acceleration weith matrix
R_acc = ca.diagcat(Acc_R1, Acc_R2, Acc_R1, Acc_R2)

# Define kinematics of the systems
leader_kin = ca.vertcat(v_l*cos(theta_l), v_l*sin(theta_l), v_l/L * tan(alpha*last_psi_l + (1-alpha)*psi_l))
motion_model_l = ca.Function('motion_model_leader', [leader_states, leader_controls, last_controls_l, L], [leader_kin])

follower_kin = ca.vertcat(v_f*cos(theta_f), v_f*sin(theta_f), v_f/L * tan(alpha*last_psi_f + (1-alpha)*psi_f))
motion_model_f = ca.Function('motion_model_follower', [follower_states, follower_controls, last_controls_f, L], [follower_kin])

theta_to_so2 = ca.Function('theta2rotm', [theta_l], [rot_2d_z])

cost_fn = 0  # cost function
g = X[:, 0] - ca.vertcat(init_pose_leader, init_pose_follower)  # constraints in the equation


def so2_error(ref, current):
    rel_m = theta_to_so2(ref).T @ theta_to_so2(current)
    return ca.atan2(rel_m[1, 0], rel_m[0, 0])

def calc_cost(ref, X, con, k, cost):
    dx = X[0, k+1] - follower_ref_poses[n_states*k]
    dy = X[1, k+1] - follower_ref_poses[n_states*k + 1]
    theta_ref = follower_ref_poses[n_states*k + 2]
    e_lat = -sin(theta_ref)*dx + cos(theta_ref)*dy
    e_lon = cos(theta_ref)*dx + sin(theta_ref)*dy
    cost += Q_lat * e_lat**2 + Q_lon * e_lon**2 + Q_theta*so2_error(theta_ref, X[2, k+1])**2 + con.T @ R @ con
    return cost

def calc_state_cost(state, ref_pose):
    dx = state[0] - ref_pose[0]
    dy = state[1] - ref_pose[1]
    theta_ref = ref_pose[2]
    e_lat = -sin(theta_ref)*dx + cos(theta_ref)*dy
    e_lon = cos(theta_ref)*dx + sin(theta_ref)*dy
    return Q_lat * e_lat**2 + Q_lon * e_lon**2 + Q_theta*so2_error(theta_ref, X[2, k+1])**2 + con.T @ R @ con

#Leader base path tracking costs
#for initial leader
k = 0
st = X[:n_states, k]
con = U[:n_controls, k]
last_vel = measured_velo_leader
st_next = X[:n_states, k+1]
cost_fn = calc_state_cost(st_next, leader_ref_poses[n_states*k:n_states*(k+1)])
k1 = motion_model_l(st, con, last_vel, L)
k2 = motion_model_l(st + step_horizon/2*k1, con, last_vel, L)
k3 = motion_model_l(st + step_horizon/2*k2, con, last_vel, L)
k4 = motion_model_l(st + step_horizon * k3, con, last_vel, L)
st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))

# runge kutta
for k in range(1, N):
    st = X[:n_states, k]
    st_next = X[:n_states, k+1]

    con = U[:n_controls, k]
    last_vel = ca.vertcat(U[0, k-1], (1 - alpha) * U[1, k-1] + alpha * last_vel[1])

    cost_fn += calc_state_cost(st_next, leader_ref_poses[n_states*k:n_states*(k+1)])
    k1 = motion_model_l(st, con, last_vel, L)
    k2 = motion_model_l(st + step_horizon/2*k1, con, last_vel, L)
    k3 = motion_model_l(st + step_horizon/2*k2, con, last_vel, L)
    k4 = motion_model_l(st + step_horizon * k3, con, last_vel, L)
    st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
    g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))

#Follower base path tracking costs
#for initial follower
k = 0
st = X[n_states:, k]
con = U[n_controls:, k]
last_vel = measured_velo_follower
st_next = X[n_states:, k+1]
cost_fn = calc_state_cost(st_next, follower_ref_poses[n_states*k:n_states*(k+1)])
k1 = motion_model_f(st, con, last_vel, L)
k2 = motion_model_f(st + step_horizon/2*k1, con, last_vel, L)
k3 = motion_model_f(st + step_horizon/2*k2, con, last_vel, L)
k4 = motion_model_f(st + step_horizon * k3, con, last_vel, L)
st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))

# runge kutta
for k in range(1, N):
    st = X[n_states:, k]
    st_next = X[n_states:, k+1]

    con = U[n_controls:, k]
    last_vel = ca.vertcat(U[n_controls, k-1], (1 - alpha) * U[n_controls+1, k-1] + alpha * last_vel[1])

    cost_fn += calc_state_cost(st_next, follower_ref_poses[n_states*k:n_states*(k+1)])
    k1 = motion_model_f(st, con, last_vel, L)
    k2 = motion_model_f(st + step_horizon/2*k1, con, last_vel, L)
    k3 = motion_model_f(st + step_horizon/2*k2, con, last_vel, L)
    k4 = motion_model_f(st + step_horizon * k3, con, last_vel, L)
    st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    g = ca.vertcat(g, st_next[:2] - st_next_RK4[:2])
    g = ca.vertcat(g, so2_error(st_next[2], st_next_RK4[2]))


# for ref_theta in [leader_ref_poses[2::3], follower_ref_poses[2::3]]:
#     g = ca.vertcat(g, ca.vertcat(-sin(ref_theta), cos(ref_theta)).T @ (X[:2, k] - P[n_states*(k+1): n_states*(k+1)+2]))

#Acceleration constraints
cost_fn += (U[:, 0] - ca.vertcat(measured_velo_leader, measured_velo_follower)).T @ R_acc @ (U[:, 0] - ca.vertcat(measured_velo_leader, measured_velo_follower))
g = ca.vertcat(g, U[:, 0])
for k in range(1, N-1):
    cost_fn += (U[:, k] - U[:, k-1]).T @ R_acc @ (U[:, k] - U[:, k-1])
    # Add acceleration constraints
    g = ca.vertcat(g, U[:, k] - U[:, k-1])

#Following constraints
for k in range(0, N):
    leader_st_next = X[:n_states, k+1]
    follower_st_next = X[n_states:, k+1]
    cost_fn += Q_dist * ca.norm_2((leader_st_next[:2] - follower_st_next[:2]))**2

    g = ca.vertcat(g, ca.norm_2((leader_st_next[:2] - follower_st_next[:2])))

# Terminal cost
# cost_fn = calc_cost(P, X, con, N-1, cost_fn)

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

solver = ca.nlpsol('solve_bicycle_joint_mpc', 'ipopt', nlp_prob, opts)
