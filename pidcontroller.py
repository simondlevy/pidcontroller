#!/usr/bin/env python3
'''
Simple altitude-hold PID controller

Copyright (C) 2019 Simon D. Levy

MIT License
'''

class AltitudePidController(object):

    def __init__(self, target, posP, velP, velI, velD, windupMax=10):

        # In a real PID controller, this would be a set-point
        self.target = target

        # Constants
        self.posP = posP
        self.velP = velP
        self.velI = velI
        self.velD = velD
        self.windupMax = windupMax

        # Values modified in-flight
        self.posTarget      = 0
        self.lastError      = 0
        self.integralError  = 0
        self.altitudeTarget = 0

    def u(self, alt, vel, dt):

        # compute dzdt setpoint and error
        velTarget = (self.target - alt) * self.posP
        velError = velTarget - vel

        # Update error integral and error derivative
        self.integralError +=  velError * dt
        self.integralError = AltitudePidController._constrainAbs(self.integralError + velError * dt, self.windupMax)
        deltaError = (velError - self.lastError) / dt if abs(self.lastError) > 0 else 0
        self.lastError = velError

        # Compute control u
        return self.velP * velError + self.velD * deltaError + self.velI * self.integralError

    def _constrainAbs(x, lim):

        return -lim if x < -lim else (+lim if x > +lim else x)


def plot(logfilename):

    data = np.genfromtxt(logfilename, delimiter=',')

    t = data[:,0]
    z = data[:,3]

    plt.plot(t, z)
    plt.xlabel('time (sec)')
    plt.ylabel('altitude (m)')
    plt.ylim([0,100])
    plt.show()
        

if __name__ == '__main__':


    import numpy as np
    import matplotlib.pyplot as plt

    # Can't touch this!
    G = 9.80665

    # Reasonable time constant
    DT = 0.001

    # Constant to experiment with
    ALTITUDE_START  = 55
    ALTITUDE_TARGET = 50

    # initial conditions
    t     = 0
    z     = ALTITUDE_START
    dzdt  = 0
    u     = 0
    zprev = 0

    # PID params
    ALT_P = 5
    VEL_P = 1.5
    VEL_I = 1.0
    VEL_D = 0.05

    # make CSV file name from these params
    filename = '%04.f-%04.f_%3.3f-%3.3f-%3.3f-%3.3f.csv' % (ALTITUDE_START, ALTITUDE_TARGET, ALT_P, VEL_P, VEL_I, VEL_D)
    logfile = open(filename, 'w')
    logfile.write('t, dzdt2, dzdt, z, u\n')

    pid = AltitudePidController(ALTITUDE_TARGET, ALT_P, VEL_P, VEL_I, VEL_D)

    while True:

        # If altitude has leveled off, halt
        if abs(z-zprev) < .0000001:
            break

        zprev = z

        dzdt2 = G - u
        
        dzdt -= dzdt2 * DT

        z += dzdt*DT

        u = pid.u(z, dzdt, DT)

        logfile.write('%3.3f,%3.3f,%3.3f,%3.3f,%3.3f\n' % (t, dzdt2, dzdt, z, u))

        t += DT

    logfile.close()

    plot(filename)


        
