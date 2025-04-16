import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from time import time


def simulate_point_stab(cat_states, cat_controls, t, step_horizon, N, reference, save=False):
    def create_triangle(state=[0,0,0], h=1, w=0.5, update=False):
        x, y, th = state
        triangle = np.array([
            [h, 0   ],
            [0,  w/2],
            [0, -w/2],
            [h, 0   ]
        ]).T
        rotation_matrix = np.array([
            [cos(th), -sin(th)],
            [sin(th),  cos(th)]
        ])

        coords = np.array([[x, y]]) + (rotation_matrix @ triangle).T
        if update == True:
            return coords
        else:
            return coords[:3, :]

    def init():
        return path, horizon, current_state, target_state,

    def animate(i):
        # get variables
        x = cat_states[0, 0, i]
        y = cat_states[1, 0, i]
        th = cat_states[2, 0, i]

        # update path
        if i == 0:
            path.set_data(np.array([]), np.array([]))
        x_new = np.hstack((path.get_xdata(), x))
        y_new = np.hstack((path.get_ydata(), y))
        path.set_data(x_new, y_new)

        # update horizon
        x_new = cat_states[0, :, i]
        y_new = cat_states[1, :, i]
        horizon.set_data(x_new, y_new)

        # update current_state
        current_state.set_xy(create_triangle([x, y, th], update=True))

        # update target_state
        # xy = target_state.get_xy()
        # target_state.set_xy(xy)            

        return path, horizon, current_state, target_state,

    # create figure and axes
    fig, ax = plt.subplots(figsize=(6, 6))
    min_scale = min(reference[0], reference[1], reference[3], reference[4]) - 2
    max_scale = max(reference[0], reference[1], reference[3], reference[4]) + 2
    ax.set_xlim(left = min_scale, right = max_scale)
    ax.set_ylim(bottom = min_scale, top = max_scale)

    # create lines:
    #   path
    path, = ax.plot([], [], 'k', linewidth=2)
    #   horizon
    horizon, = ax.plot([], [], 'x-g', alpha=0.5)
    #   current_state
    current_triangle = create_triangle(cat_states[:3, 0, 0])
    current_state = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='r')
    current_state = current_state[0]
    #   target_state
    target_triangle = create_triangle(reference[3:])
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]

    sim = animation.FuncAnimation(
        fig=fig,
        func=animate,
        init_func=init,
        frames=len(t),
        interval=step_horizon*100,
        blit=True,
        repeat=True
    )
    plt.show()

    if save == True:
        sim.save('./animation' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return


def simulate_path_tracking(cat_states, cat_controls, t, step_horizon, N, reference_path, save=False):
    def create_triangle(state=[0,0,0], h=1, w=0.5, update=False):
        x, y, th = state
        triangle = np.array([
            [h, 0   ],
            [0,  w/2],
            [0, -w/2],
            [h, 0   ]
        ]).T
        rotation_matrix = np.array([
            [cos(th), -sin(th)],
            [sin(th),  cos(th)]
        ])

        coords = np.array([[x, y]]) + (rotation_matrix @ triangle).T
        if update == True:
            return coords
        else:
            return coords[:3, :]

    def init():
        return path, horizon, current_state, target_state,

    def animate(i):
        # get variables
        x = cat_states[0, 0, i]
        y = cat_states[1, 0, i]
        th = cat_states[2, 0, i]

        # update path
        if i == 0:
            path.set_data(np.array([]), np.array([]))
        x_new = np.hstack((path.get_xdata(), x))
        y_new = np.hstack((path.get_ydata(), y))
        path.set_data(x_new, y_new)

        # update horizon
        x_new = cat_states[0, :, i]
        y_new = cat_states[1, :, i]
        horizon.set_data(x_new, y_new)

        # update current_state
        current_state.set_xy(create_triangle([x, y, th], update=True))

        # update target_state
        # xy = target_state.get_xy()
        # target_state.set_xy(xy)            

        return path, horizon, current_state, target_state,

    # create figure and axes
    fig, ax = plt.subplots(figsize=(6, 6))
    min_scale = np.min(reference_path[:, :2]) - 2
    max_scale = np.max(reference_path[:, :2]) + 2
    ax.set_xlim(left = min_scale, right = max_scale)
    ax.set_ylim(bottom = min_scale, top = max_scale)

    # create lines:
    #   path
    path, = ax.plot([], [], 'k', linewidth=2)
    #   horizon
    horizon, = ax.plot([], [], 'x-g', alpha=0.5)
    #   current_state
    current_triangle = create_triangle(cat_states[:3, 0, 0])
    current_state = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='r')
    current_state = current_state[0]
    #   target_state
    target_triangle = create_triangle(reference_path[-1])
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]
    ref_path, = ax.plot(reference_path[:, 0], reference_path[:, 1], color='g', label="Ref Path")

    sim = animation.FuncAnimation(
        fig=fig,
        func=animate,
        init_func=init,
        frames=len(t),
        interval=step_horizon*100,
        blit=True,
        repeat=True
    )
    plt.show()

    if save == True:
        sim.save('./animation' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return


def simulate_path_tracking_convoy(cat_states_1, cat_controls_1, cat_states_2, cat_controls_2, t, step_horizon, N, reference_path, save=False):
    def create_triangle(state=[0,0,0], h=1, w=0.5, update=False):
        x, y, th = state
        triangle = np.array([
            [h, 0   ],
            [0,  w/2],
            [0, -w/2],
            [h, 0   ]
        ]).T
        rotation_matrix = np.array([
            [cos(th), -sin(th)],
            [sin(th),  cos(th)]
        ])

        coords = np.array([[x, y]]) + (rotation_matrix @ triangle).T
        if update == True:
            return coords
        else:
            return coords[:3, :]

    def init():
        return path_1, path_2, horizon_1, horizon_2, current_state_1, current_state_2, target_state,

    def animate(i):
        # get variables for first set
        x1 = cat_states_1[0, 0, i]
        y1 = cat_states_1[1, 0, i]
        th1 = cat_states_1[2, 0, i]

        # get variables for second set
        x2 = cat_states_2[0, 0, i]
        y2 = cat_states_2[1, 0, i]
        th2 = cat_states_2[2, 0, i]

        # update path for first set
        if i == 0:
            path_1.set_data(np.array([]), np.array([]))
        x_new_1 = np.hstack((path_1.get_xdata(), x1))
        y_new_1 = np.hstack((path_1.get_ydata(), y1))
        path_1.set_data(x_new_1, y_new_1)

        # update path for second set
        if i == 0:
            path_2.set_data(np.array([]), np.array([]))
        x_new_2 = np.hstack((path_2.get_xdata(), x2))
        y_new_2 = np.hstack((path_2.get_ydata(), y2))
        path_2.set_data(x_new_2, y_new_2)

        # update horizon for first set
        x_new_1 = cat_states_1[0, :, i]
        y_new_1 = cat_states_1[1, :, i]
        horizon_1.set_data(x_new_1, y_new_1)

        # update horizon for second set
        x_new_2 = cat_states_2[0, :, i]
        y_new_2 = cat_states_2[1, :, i]
        horizon_2.set_data(x_new_2, y_new_2)

        # update current_state for first set
        current_state_1.set_xy(create_triangle([x1, y1, th1], update=True))

        # update current_state for second set
        current_state_2.set_xy(create_triangle([x2, y2, th2], update=True))

        return path_1, path_2, horizon_1, horizon_2, current_state_1, current_state_2, target_state,

    # create figure and axes
    fig, ax = plt.subplots(figsize=(6, 6))
    min_scale = np.min(reference_path[:, :2]) - 2
    max_scale = np.max(reference_path[:, :2]) + 2
    ax.set_xlim(left = min_scale, right = max_scale)
    ax.set_ylim(bottom = min_scale, top = max_scale)
    ax.set_aspect('equal')

    # create lines:
    #   path for first set
    path_1, = ax.plot([], [], 'k', linewidth=2)
    #   path for second set
    path_2, = ax.plot([], [], 'b', linewidth=2)
    #   horizon for first set
    horizon_1, = ax.plot([], [], 'x-g', alpha=0.5)
    #   horizon for second set
    horizon_2, = ax.plot([], [], 'x-r', alpha=0.5)
    #   current_state for first set
    current_triangle_1 = create_triangle(cat_states_1[:3, 0, 0])
    current_state_1 = ax.fill(current_triangle_1[:, 0], current_triangle_1[:, 1], color='r')
    current_state_1 = current_state_1[0]
    #   current_state for second set
    current_triangle_2 = create_triangle(cat_states_2[:3, 0, 0])
    current_state_2 = ax.fill(current_triangle_2[:, 0], current_triangle_2[:, 1], color='b')
    current_state_2 = current_state_2[0]
    #   target_state
    target_triangle = create_triangle(reference_path[-1])
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='g')
    target_state = target_state[0]
    ref_path, = ax.plot(reference_path[:, 0], reference_path[:, 1], color='g', label="Ref Path")

    sim = animation.FuncAnimation(
        fig=fig,
        func=animate,
        init_func=init,
        frames=len(t),
        interval=step_horizon*100,
        blit=True,
        repeat=True
    )
    plt.show()

    if save == True:
        sim.save('/home/sven/ASRL/vtr3/animation' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return