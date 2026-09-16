import casadi as ca

from bicycle_follower_joint_solver_pt import solver, N, step_horizon, n_states, n_controls


# specs
x_init_l = -0.12
y_init_l = 0
theta_init_l = 0

x_init_f = -3.3
y_init_f = 0
theta_init_f = 0

dist = 1.5

v_max = 1.5
v_min = -1.5
theta_max = 0.5
theta_min = -0.5
v_ref = 0.5*v_max

lin_acc_max = 340
ang_vel_max = 0.89

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

for leader_roll in ([0.0662941, 0.00109525, 0.00250014], [0.243053, 0.00482211, 0.00916682], [0.418625, 0.0076441, 0.0167252],
        [0.592683, 0.00916518, 0.0254463], [0.76598, 0.0122822, 0.0362155], [0.938281, 0.0175487, 0.0494577],
        [1.11333, 0.0287706, 0.0543499], [1.28976, 0.0422937, 0.0551521], [1.46166, 0.0530624, 0.0502071],
        [1.632, 0.0618178, 0.0432793], [1.80916, 0.0701468, 0.0486206], [1.98573, 0.0793484, 0.054951],
        [2.15941, 0.0890223, 0.062203], [2.33314, 0.102301, 0.0633594], [2.50715, 0.119128, 0.0535195]):
   p = ca.vertcat(p, ca.DM(leader_roll))

# for x_l in range(1, N+1):
#     p = ca.vertcat(p, ca.DM([x_init_l + v_ref*x_l*step_horizon, 0, 0]))
p = ca.vertcat(p, ca.DM([0.0, 0]))

# Follower initial state
p = ca.vertcat(p, ca.DM([x_init_f, y_init_f, theta_init_f]))
for follower_roll in ([-1.42679, -0.00211212, -0.0107171], [-1.25418, -0.00345247, -0.00776228], [-1.08118, -0.00453858, -0.00490558],
    [-0.902415, -0.00417166, -0.00593979], [-0.726319, -0.00369981, -0.00791078], [-0.561893, -0.00419456, -0.00873041],
    [-0.377337, -0.00502756, -0.00936899], [-0.21177, -0.00359067, -0.00582076], [-0.0462898, -0.000895048, -0.00127211],
    [0.141096, 0.00252916, 0.00532126], [0.310775, 0.00656107, 0.0117213], [0.478355, 0.00799672, 0.019718], [0.664016, 0.0102228, 0.0290204],
    [0.829717, 0.0139693, 0.0411129], [1.01391, 0.0212091, 0.0538975]):
    p = ca.vertcat(p, ca.DM(follower_roll))
# for x_f in range(1, N+1):
#     p = ca.vertcat(p, ca.DM([x_init_f + v_ref*x_f*step_horizon, 0, 0]))
p = ca.vertcat(p, ca.DM([0.0, 0]))
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
p = ca.vertcat(p, ca.DM(1)) # Q_dist

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
u_l = u[:n_controls, :]
u_f = u[n_controls:, :]

X0 = ca.reshape(joint_sol['x'][:2 * n_states * (N+1)], 2*n_states, N+1)
X0_l = X0[:n_states, :]
X0_f = X0[n_states:, :]
print(X0_l[0, :])
print(X0_f[0, :])

print(u_l[0, :])
print(u_f[0, :])