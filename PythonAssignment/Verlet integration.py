# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100
dt = 0.1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = [0, 0.1]
v_list = []

# Verlet integration
for i in range(len(t_array)):

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate new position and velocity
    a = -k * x_list[i+1] / m
    x = 2 * x_list[i+1] - x_list[i] + dt**2 * a
    v = 1 / dt *(x_list[i+1] - x_list[i])

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x1_array = np.array(x_list)
x_array = x1_array[1:]
v_array = np.array(v_list)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()

