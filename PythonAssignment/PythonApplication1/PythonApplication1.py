
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
x_list = []
v_list = []
x1_list = []
v1_list = []
# Euler integration
for t in t_array:

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate new position and velocity
    a = -k * x / m
    x = x + dt * v
    v = v + dt * a

#Verlet Integration
leng = len(t_array)
x1_list.append(0)
x = 0
v = 1

for i in range(leng):
    x1_list.append(x)
    v1_list.append(v)

    a = -k * x1_list[i+1] / m
    x = 2 * x1_list[i+1] - x1_list[i] + dt * dt * a
    v = 1 / dt * (x1_list[i+1] - x1_list[i])


# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)
x1_array = np.array(x1_list)
x2_array = x1_array[1:]
v1_array = np.array(v1_list)


# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()


plt.figure(2)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x2_array, label='x (m)')
plt.plot(t_array, v1_array, label='v (m/s)')
plt.legend()
plt.show()
