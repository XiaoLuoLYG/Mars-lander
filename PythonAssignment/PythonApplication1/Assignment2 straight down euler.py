
# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
mars_radius = 3395000
r = [0, 0, 3.4e6]
v = [0, 0, 0]
G = 6.67408e-11
M = 6.42e23
position = np.array(r)
velocity = np.array(v)


# simulation time, timestep and time
t_max = 1000
dt = 0.01

t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
r_list = []
v_list = []

# Euler integration
for i in range(len(t_array)):

    # append current state to trajectories
    r_list.append(position)
    v_list.append(velocity)

    p_modules = np.linalg.norm(position,ord=None)
    p_normal = position / p_modules

    # calculate new position and velocity
    a = - (G * M / p_modules**2) * p_normal
    position = position + dt * velocity
    velocity = velocity + dt * a

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)

r_array = np.array(r_list)
v_array = np.array(v_list)
z_list = []
vz_list = []
for r in r_array:
    z_list.append(r[-1])

for v in v_array:
    vz_list.append(v[-1])

z_array = np.array(z_list)
vz_array = np.array(vz_list)
print(vz_array)
print(r_array)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, z_array, label='z (m)')
plt.legend()
plt.show()

