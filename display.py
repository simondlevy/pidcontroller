#!/usr/bin/env python3
'''
Display CSV log file from pidcontroller.py:

t, dzdt2, dzdt, z, u
'''

import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('tmp.csv', delimiter=',')

t = data[:,0]
z = data[:,3]

plt.plot(t, z)
plt.xlabel('time (sec)')
plt.ylabel('altitude (m)')
plt.show()
