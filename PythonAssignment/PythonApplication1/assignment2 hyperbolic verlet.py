

# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import random

# mass,initial position and velocity
m = 1
mars_radius = 3395000
r = [3.396e6, 0, 0]
G = 6.67408e-11
M = 6.42e23
v_orbit = (G * M / r[0]) ** 0.5
v_escape = (2 * G * M / r[0]) ** 0.5
v0 = random.uniform(v_escape, v_escape + 1000)
v = [0, v0, 0]
position = np.array(r)
velocity = np.array(v)

# simulation time, timestep and time
t_max = 3000
dt = 0.01

t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
r_list = [position - dt * velocity]
v_list = []

# Verlet integration
for i in range(len(t_array)):

    # append current state to trajectories
    r_list.append(position)
    v_list.append(velocity)

    p_modules = np.linalg.norm(position,ord=None)
    p_normal = position / p_modules

    # calculate new position and velocity
    a = - (G * M / p_modules**2) * p_normal
    position = 2 * position - r_list[-2] + (dt ** 2) * a
    velocity = 0.5 / dt * (position - r_list[-1])

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)

r_array = np.array(r_list[1:])
v_array = np.array(v_list)
x_list = []
vx_list = []
y_list =[]

for r in r_array:
    x_list.append(r[0])
    y_list.append(r[1])

for v in v_array:
    vx_list.append(v[0])

x_array = np.array(x_list)
y_array = np.array(y_list)
vx_array = np.array(vx_list)

print(vx_array)
print(r_array)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('x(m)')
plt.ylabel('y(m)')
plt.grid()
plt.plot(x_array, y_array, label='Trajectory')
plt.legend()
plt.show()

plt.figure(2)
plt.clf()
plt.xlabel('t(s)')
plt.grid()
plt.plot(t_array, vx_array, label='vx (m)')
plt.legend()
plt.show()


