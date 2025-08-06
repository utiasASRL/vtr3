from matplotlib import pyplot as plt
import numpy as np

sim_dt = 0.1
t = np.arange(0, 100, sim_dt)

D = 2.0

A = 0.56
A0 = 2.5

leader_x = A*t+A0


dt = 0.1
follower_x = [0]
x_vel = 0
Kp = 2.0
Ki = 0.5
Kd = 1.0

vel_max = 1.5

err_int = 0
last_error = 0
alpha = 0.85

for lx in leader_x[:-1]:
    err_i = lx - follower_x[-1] - D + np.random.normal(0, 0.0)
    err_int += err_i * dt
    cmd_vel = Kp * err_i + dt * Ki * err_int + Kd * (err_i - last_error) / dt
    if abs(cmd_vel) > vel_max:
        print("Exceeded max velo!")
        cmd_vel = np.sign(cmd_vel) * vel_max
    x_vel = alpha * x_vel + (1-alpha) * cmd_vel
    follower_x.append(follower_x[-1] + dt * x_vel)
    last_error = err_i

follower_x = np.array(follower_x)
error = leader_x - follower_x - D

plt.plot(t, leader_x, label="Leader")
plt.plot(t, follower_x, label="Follower")
plt.legend()

plt.figure()

plt.plot(t, error)
plt.title("Error")

# print(f"Predicted final error {A/dt/Kp:.3f}m")
print(f"Final error {np.mean(error[50:]):.3f}m")
print(f"Steady state stdev error {np.std(error[50:]):.3f}m")

plt.show()
