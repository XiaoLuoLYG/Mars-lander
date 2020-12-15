import numpy as np
import matplotlib.pyplot as plt
results1= np.loadtxt('C:/Users/scofieldluo/Desktop/trajectories kh0.0167.txt')
results2 = np.loadtxt('C:/Users/scofieldluo/Desktop/trajectories kh0.0170.txt')
results3 = np.loadtxt('C:/Users/scofieldluo/Desktop/trajectories kh0.0175.txt')

plt.figure(1)
plt.clf()
plt.xlabel('Altitude')
plt.ylabel('descent rate (m/s)')
plt.grid()
plt.plot(results1[:, 0], results1[:, 1], label = ('Kh = 0.0167'))
plt.plot(results2[:, 0], results2[:, 1], label = ('Kh = 0.0170'))
plt.plot(results3[:, 0], results3[:, 1], label = ('Kh = 0.0175'))
plt.legend()
plt.axhline(y = 1.0)
plt.show()

